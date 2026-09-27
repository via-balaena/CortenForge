//! Step 6's obstacle bake (`docs/SOFT_CONTACT_ARCHITECTURE_RECON.md` §15g step 6, §16u; fit plan U18): G2 against
//! the grid's spacing on `base_mold`, below §16r's finest, 0.25 mm; and the bake itself on it.
//!
//! A dense grid below §16r's finest spacing holds hundreds of millions of samples or more, but G2 reads only the
//! tricubic stencils around points on the scan. So this reads the scan's exact signed distance at those stencils'
//! samples alone, a band a few cells thick, and evaluates each point with the solver's own lookup
//! ([`sim_soft_explicit::f64::sdf_tricubic`]) over the layout a dense grid of the same spacing would have.
//! At a spacing both can build, the band reads what the dense grid reads
//! (`the_band_reads_what_the_dense_grid_reads`).
//!
//! The distance is the closed scan's, to its full-resolution triangles. The sign is one of four, compared: the
//! scan's pseudo-normal sign as loaded and once welded, the flood fill §16r's grids were signed by, and the parity
//! of a ray's crossings, which the bake uses.
//!
//! ⛔ The scan never enters the repo. The probes print to the terminal, and the plan records only ratios and
//! verdicts (Jon, 2026-09-26).

#![cfg(test)]
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use std::collections::HashMap;
use std::thread;

use mesh_repair::weld_vertices;
use mesh_sdf::{
    FloodFillSign, ParitySign, PseudoNormalSign, Sign, TriMeshDistance, UnsignedDistance,
};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};
use sim_soft_explicit::f64::{SdfGridLayout, sdf_grid_coordinate, sdf_tricubic, sdf_tricubic_axis};

use super::explicit_budget::g2_bar;
use super::{Aabb, scan_aabb};

/// The product's fine grid spacing (Jon, 2026-09-27; plan §16u): the one measured spacing whose error meets G2's
/// floor, so at every inset.
const PRODUCT_FINE_CELL: f64 = 0.000_062_5;

/// The layout of a dense grid of spacing `cell` over `bounds`, as §16r's grids were laid out: its last sample can
/// fall short of `bounds.max` by up to a cell, where the bake's reaches it. The lattices coincide.
fn layout(bounds: Aabb, cell: f64) -> SdfGridLayout {
    let side = |extent: f64| {
        u32::try_from((extent / cell).floor() as u64 + 1).expect("a grid side fits a u32")
    };
    SdfGridLayout {
        origin_x: bounds.min.x,
        origin_y: bounds.min.y,
        origin_z: bounds.min.z,
        cell_size: cell,
        size_x: side(bounds.max.x - bounds.min.x),
        size_y: side(bounds.max.y - bounds.min.y),
        size_z: side(bounds.max.z - bounds.min.z),
    }
}

/// The 64 samples of `point`'s tricubic stencil, as (column, row, layer), in the order the lookup reads them.
fn stencil(point: Point3<f64>, grid: SdfGridLayout) -> ([f64; 3], [[u32; 3]; 64]) {
    let coordinate = sdf_grid_coordinate([point.x, point.y, point.z], grid);
    let columns = sdf_tricubic_axis(coordinate[0], grid.size_x);
    let rows = sdf_tricubic_axis(coordinate[1], grid.size_y);
    let layers = sdf_tricubic_axis(coordinate[2], grid.size_z);
    let mut samples = [[0; 3]; 64];
    for (index, sample) in samples.iter_mut().enumerate() {
        *sample = [columns[index % 4], rows[index / 4 % 4], layers[index / 16]];
    }
    (coordinate, samples)
}

/// The position of a grid sample.
fn at(sample: [u32; 3], grid: SdfGridLayout) -> Point3<f64> {
    Point3::new(
        grid.origin_x + f64::from(sample[0]) * grid.cell_size,
        grid.origin_y + f64::from(sample[1]) * grid.cell_size,
        grid.origin_z + f64::from(sample[2]) * grid.cell_size,
    )
}

/// `signed` at every sample of every point's stencil, computed once per sample, on every core.
fn band_values(
    points: &[Point3<f64>],
    grid: SdfGridLayout,
    signed: &(impl Fn(Point3<f64>) -> f64 + Sync),
) -> HashMap<[u32; 3], f64> {
    let mut keys: Vec<[u32; 3]> = points.iter().flat_map(|p| stencil(*p, grid).1).collect();
    keys.sort_unstable();
    keys.dedup();
    let threads = thread::available_parallelism().map_or(1, usize::from);
    let chunk = keys.len().div_ceil(threads).max(1);
    thread::scope(|scope| {
        let handles: Vec<_> = keys
            .chunks(chunk)
            .map(|part| {
                scope.spawn(move || {
                    part.iter()
                        .map(|key| (*key, signed(at(*key, grid))))
                        .collect::<Vec<_>>()
                })
            })
            .collect();
        handles
            .into_iter()
            .flat_map(|handle| handle.join().unwrap())
            .collect()
    })
}

/// The grid's distance at `point`, read from the band as the solver's lookup reads a dense grid.
fn read(point: Point3<f64>, grid: SdfGridLayout, band: &HashMap<[u32; 3], f64>) -> f64 {
    let (coordinate, samples) = stencil(point, grid);
    let values = samples.map(|sample| band[&sample]);
    sdf_tricubic(coordinate, values, grid).distance
}

/// The points G2 reads on a surface: each vertex its faces name, as often as it is stored (a soup stores a
/// position once per face that has it), and each face's centroid.
fn surface_points(surface: &IndexedMesh) -> Vec<Point3<f64>> {
    let mut named: Vec<u32> = surface.faces.iter().flatten().copied().collect();
    named.sort_unstable();
    named.dedup();
    let centroids = surface.faces.iter().map(|face| {
        let [a, b, c] = face.map(|v| surface.vertices[v as usize].coords);
        Point3::from((a + b + c) / 3.0)
    });
    named
        .iter()
        .map(|&v| surface.vertices[v as usize])
        .chain(centroids)
        .collect()
}

/// The `q` quantile of sorted values (nearest rank).
fn quantile(sorted: &[f64], q: f64) -> f64 {
    let rank = ((sorted.len() - 1) as f64 * q).round() as usize;
    sorted[rank]
}

#[test]
fn the_band_reads_what_the_dense_grid_reads() {
    // An analytic field, a sphere's distance, on a grid small enough to build dense: the band holds only the
    // stencils' samples, and every point reads the same distance from it as from the dense grid.
    let (centre, radius) = (Point3::new(0.1, -0.2, 0.05), 0.6);
    let signed = move |p: Point3<f64>| (p - centre).norm() - radius;
    let bounds = Aabb::new(Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0));
    let grid = layout(bounds, 0.05);
    let points: Vec<Point3<f64>> = (0..200)
        .map(|k| {
            let (a, b) = (f64::from(k) * 0.37, f64::from(k) * 0.113);
            centre + Vector3::new(a.cos() * b.sin(), a.sin() * b.sin(), b.cos()) * radius
        })
        .collect();
    let band = band_values(&points, grid, &signed);
    let mut dense = Vec::new();
    for layer in 0..grid.size_z {
        for row in 0..grid.size_y {
            for column in 0..grid.size_x {
                dense.push(signed(at([column, row, layer], grid)));
            }
        }
    }
    let obstacle = sim_soft_explicit::executor::Obstacle {
        grid,
        values: dense,
        fine: None,
        start: 0.0,
        interval: 1.0,
        poses: vec![sim_soft_explicit::f64::Pose {
            qw: 1.0,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            tx: 0.0,
            ty: 0.0,
            tz: 0.0,
        }],
        friction: 0.0,
    };
    assert!(band.len() < (grid.size_x * grid.size_y * grid.size_z) as usize / 4);
    for p in &points {
        let dense = obstacle.sample([p.x, p.y, p.z]).distance;
        assert!((read(*p, grid, &band) - dense).abs() < 1e-15, "{p}");
    }
}

/// A sign read near the scan: which side of it a point is on.
type Side<'a> = Box<dyn Fn(Point3<f64>) -> bool + Sync + 'a>;

/// The scan's distance, signed by `side` (true inside).
fn signed_by<'a>(
    closed: &'a TriMeshDistance,
    side: &'a (dyn Fn(Point3<f64>) -> bool + Sync),
) -> impl Fn(Point3<f64>) -> f64 + Sync + 'a {
    move |p| {
        let d = closed.distance(p);
        if side(p) { -d } else { d }
    }
}

/// G2 against the grid's spacing on the product: the grid's distance at points on the scan, where it should read
/// zero, as §16r's table reads it (positive: the grid's surface lies inside the scan, and a node the contact law
/// holds there sits that deep in it). Over every point and over all but the cap discs.
///
/// First, the signs at 0.25 mm's band: the scan's pseudo-normal sign as loaded (a triangle soup, three vertices a
/// face) and once welded, the flood fill §16r's grids were signed by, and parity. Four pairs' disagreements (each
/// against parity, and the welded pseudo-normals against the flood fill) are binned by distance from the surface.
/// Then G2 with the flood fill's sign at 0.25 mm, compared by eye with §16r's row off the cap discs (the check on
/// the band), and with the welded pseudo-normal sign and parity at three spacings.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn g2_against_the_grids_spacing_on_the_product_scan() {
    let (scan, _, caps, design) =
        super::tests::product_scene().expect("the bake is measured on the product scan");
    let bar = g2_bar(design.cavity_inset_m);
    let bounds = scan_aabb(&scan, 0.004);
    let closed = TriMeshDistance::new(scan.clone()).expect("the scan's distance must build");
    let mut welded = scan.clone();
    let merged = weld_vertices(&mut welded, 1e-6);
    let welded_distance =
        TriMeshDistance::new(welded.clone()).expect("the welded scan's distance must build");
    let (soup_health, welded_health) = (closed.health(), welded_distance.health());
    println!(
        "\n══ step 6's bake: G2 against the grid's spacing · base_mold · bar {:.4} mm [LOCAL] ══\n\
         welding merged {merged} of {} vertices; zero pseudo-normals, soup: {} vertices, {} edges; welded: {} \
         vertices, {} edges; faces skipped {} / {}",
        bar * 1e3,
        scan.vertices.len(),
        soup_health.zero_pseudo_normal_vertices,
        soup_health.zero_pseudo_normal_edges,
        welded_health.zero_pseudo_normal_vertices,
        welded_health.zero_pseudo_normal_edges,
        soup_health.faces_skipped,
        welded_health.faces_skipped,
    );
    let (soup_sign, welded_sign) = (
        PseudoNormalSign::from_distance(&closed),
        PseudoNormalSign::from_distance(&welded_distance),
    );

    // The signs at 0.25 mm's band.
    let cell = 0.000_25;
    let grid = layout(bounds, cell);
    let (flood, _) =
        FloodFillSign::build(&closed, bounds, cell, 0.75).expect("the flood fill must build");
    // Parity reads only the triangles, which the soup and the exactly welded scan share: this is the bake's sign.
    let parity = ParitySign::new(&scan).expect("the scan must bin");
    let sides: [(&str, Side<'_>); 4] = [
        ("soup pseudo-normal", Box::new(|p| soup_sign.is_inside(p))),
        (
            "welded pseudo-normal",
            Box::new(|p| welded_sign.is_inside(p)),
        ),
        ("flood fill", Box::new(|p| flood.is_inside(p))),
        ("parity", Box::new(|p| parity.is_inside(p))),
    ];
    let all = surface_points(&scan);
    let mut keys: Vec<[u32; 3]> = all.iter().flat_map(|p| stencil(*p, grid).1).collect();
    keys.sort_unstable();
    keys.dedup();
    let readings: Vec<(Point3<f64>, f64, [bool; 4])> = keys
        .iter()
        .map(|key| {
            let p = at(*key, grid);
            (
                p,
                closed.distance(p) / cell,
                [0, 1, 2, 3].map(|k| sides[k].1(p)),
            )
        })
        .collect();
    println!(
        "signs at 0.25 mm's band ({} samples): disagreements by distance from the surface, in cells",
        readings.len()
    );
    // Each pair of signs, and where each disagrees with parity.
    for (a, b) in [(1, 2), (1, 3), (2, 3), (0, 3)] {
        let bins = [0.25, 0.5, 1.0, f64::INFINITY];
        let mut counts = [0_usize; 4];
        for (_, d, side) in &readings {
            if side[a] != side[b] {
                counts[bins.iter().position(|edge| d < edge).unwrap()] += 1;
            }
        }
        println!(
            "  {} vs {}: < 0.25 {} · 0.25–0.5 {} · 0.5–1 {} · ≥ 1 {}",
            sides[a].0, sides[b].0, counts[0], counts[1], counts[2], counts[3]
        );
    }

    let sides_of_the_scan = cf_cap_planes::dome_wall_only_mesh(&scan, &caps);
    let planes: Vec<(Point3<f64>, Vector3<f64>)> =
        caps.iter().map(cf_cap_planes::CapPlane::as_tuple).collect();
    let from_the_caps = |p: Point3<f64>| {
        planes
            .iter()
            .map(|(c, n)| (p - c).dot(&n.normalize()).abs())
            .fold(f64::INFINITY, f64::min)
    };
    let off_the_discs = surface_points(&sides_of_the_scan);
    let runs: [(&str, usize, &[f64]); 3] = [
        ("flood fill", 2, &[0.000_25]),
        (
            "welded pseudo-normal",
            1,
            &[0.000_25, 0.000_125, 0.000_062_5],
        ),
        ("parity", 3, &[0.000_25, 0.000_125, 0.000_062_5]),
    ];
    for (name, side, cells) in runs {
        for &cell in cells {
            let grid = layout(bounds, cell);
            let started = std::time::Instant::now();
            let band = band_values(&all, grid, &signed_by(&closed, &*sides[side].1));
            let dense = u64::from(grid.size_x) * u64::from(grid.size_y) * u64::from(grid.size_z);
            println!(
                "{name}, grid {:.4} mm: band {} samples, {:.3} % of the dense grid's {dense}; {:.1} s [LOCAL]",
                cell * 1e3,
                band.len(),
                100.0 * band.len() as f64 / dense as f64,
                started.elapsed().as_secs_f64()
            );
            for (label, points) in [
                ("all points", &all),
                ("all but the cap discs", &off_the_discs),
            ] {
                let readings: Vec<(Point3<f64>, f64)> = points
                    .iter()
                    .map(|p| (*p, read(*p, grid, &band) / bar))
                    .collect();
                let sorted = |part: fn(f64) -> f64| {
                    let mut values: Vec<f64> = readings.iter().map(|&(_, r)| part(r)).collect();
                    values.sort_by(f64::total_cmp);
                    values
                };
                let (penetration, gap) = (sorted(|r| r.max(0.0)), sorted(|r| (-r).max(0.0)));
                let past = readings.iter().filter(|&&(_, r)| r > 1.0).count();
                let deepest = readings
                    .iter()
                    .max_by(|a, b| a.1.total_cmp(&b.1))
                    .map(|&(p, _)| p)
                    .unwrap();
                println!(
                    "  {label}: past the bar {:.3} % | penetration over the bar p95 {:.3} p99 {:.3} p99.9 {:.3} worst \
                     {:.3}, {:.2} mm from a cap plane | shortfall p95 {:.3} worst {:.3}; {} points",
                    100.0 * past as f64 / readings.len() as f64,
                    quantile(&penetration, 0.95),
                    quantile(&penetration, 0.99),
                    quantile(&penetration, 0.999),
                    quantile(&penetration, 1.0),
                    from_the_caps(deepest) * 1e3,
                    quantile(&gap, 0.95),
                    quantile(&gap, 1.0),
                    readings.len()
                );
            }
        }
    }
}

/// Step 6's bake on the product (`sim_soft::obstacle`): a coarse grid at 0.5 mm, and a fine one at 0.25 mm and at
/// 0.0625 mm within a fine cell of the surface. G2 read through the solver's own lookup (`Obstacle::sample`), which
/// must read at every point what the band signed by parity reads at the same spacing: the samples are the same
/// points, with the same distance and sign, so it asserts they agree, and that no point is past G2's bar at the
/// product's inset. The obstacle is the same at every inset, so it prints the smallest whole-millimetre inset
/// whose bar each spacing meets, and asserts the product's spacing meets G2's floor. Beside it the bake's size
/// and time.
///
/// Parity reads a region the scan encloses twice as outside, where a flood fill reads it inside. So it also asserts
/// that the coarse grid's sign agrees with a flood fill of the same spacing at every sample a flood cell or more
/// from the surface. An overlap thinner than that holds no such sample, and is not compared.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn the_bake_on_the_product_scan() {
    let (scan, _, caps, design) =
        super::tests::product_scene().expect("the bake is measured on the product scan");
    let bar = g2_bar(design.cavity_inset_m);
    let (margin, coarse_cell) = (0.004, 0.000_5);
    let bounds = scan_aabb(&scan, margin);
    let closed = TriMeshDistance::new(scan.clone()).expect("the scan's distance must build");
    let parity = ParitySign::new(&scan).expect("the scan must bin");
    let inside = |p: Point3<f64>| parity.is_inside(p);
    let signed = signed_by(&closed, &inside);
    let (flood, _) = FloodFillSign::build(&closed, bounds, coarse_cell, 0.75)
        .expect("the flood fill must build");
    let sides = cf_cap_planes::dome_wall_only_mesh(&scan, &caps);
    let point_sets = [
        ("all points", surface_points(&scan)),
        ("all but the cap discs", surface_points(&sides)),
    ];
    println!(
        "\n══ step 6's bake on base_mold · bar {:.4} mm [LOCAL] ══",
        bar * 1e3
    );
    for (fine_cell, product) in [(0.000_25, false), (PRODUCT_FINE_CELL, true)] {
        let bake = sim_soft::obstacle::ObstacleBake {
            coarse_cell,
            fine_cell,
            band: fine_cell,
            margin,
        };
        let started = std::time::Instant::now();
        let baked =
            sim_soft::obstacle::bake_obstacle(&scan, bake).expect("the product scan must bake");
        let seconds = started.elapsed().as_secs_f64();
        let kept = baked
            .fine
            .map
            .iter()
            .filter(|&&slot| slot != sim_soft_explicit::f64::SDF_NO_BRICK)
            .count();
        println!(
            "fine {:.4} mm: baked in {seconds:.1} s; coarse {} samples; fine lattice {} × {} × {}, {kept} of {} \
             bricks, {} values ({:.0} MB at f64, {:.0} at f32)",
            fine_cell * 1e3,
            baked.values.len(),
            baked.fine.grid.size_x,
            baked.fine.grid.size_y,
            baked.fine.grid.size_z,
            baked.fine.map.len(),
            baked.fine.values.len(),
            baked.fine.values.len() as f64 * 8.0 / 1e6,
            baked.fine.values.len() as f64 * 4.0 / 1e6,
        );
        let obstacle = sim_soft_explicit::executor::Obstacle {
            grid: baked.grid,
            values: baked.values,
            fine: Some(baked.fine),
            start: 0.0,
            interval: 1.0,
            poses: vec![sim_soft_explicit::f64::Pose {
                qw: 1.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                tx: 0.0,
                ty: 0.0,
                tz: 0.0,
            }],
            friction: 0.0,
        };
        sim_soft_explicit::executor::check_obstacle(&obstacle)
            .expect("the bake must be a valid obstacle");
        let g = obstacle.grid;
        let (mut compared, mut disputed) = (0_usize, 0_usize);
        for (n, &value) in obstacle.values.iter().enumerate() {
            if value.abs() < g.cell_size {
                continue;
            }
            let (x, y) = (g.size_x as usize, g.size_y as usize);
            let p = at(
                [n % x, n / x % y, n / (x * y)].map(|i| u32::try_from(i).unwrap()),
                g,
            );
            compared += 1;
            if (value < 0.0) != flood.is_inside(p) {
                disputed += 1;
            }
        }
        println!(
            "  coarse samples a flood cell or more out: {compared}; parity against the flood fill disputed at \
             {disputed}"
        );
        assert_eq!(
            disputed, 0,
            "the coarse grid's sign and the flood fill's agree away from the surface"
        );
        let fine = obstacle.fine.as_ref().unwrap();
        let grid = layout(bounds, fine_cell);
        let band = band_values(&point_sets[0].1, grid, &signed);
        for (label, points) in &point_sets {
            let fine_reads = points
                .iter()
                .filter(|p| fine.sample([p.x, p.y, p.z]).is_some())
                .count();
            let readings: Vec<f64> = points
                .iter()
                .map(|p| obstacle.sample([p.x, p.y, p.z]).distance)
                .collect();
            let apart = points
                .iter()
                .zip(&readings)
                .map(|(p, r)| (r - read(*p, grid, &band)).abs())
                .fold(0.0, f64::max);
            let sorted = |part: fn(f64) -> f64| {
                let mut values: Vec<f64> = readings.iter().map(|r| part(r / bar)).collect();
                values.sort_by(f64::total_cmp);
                values
            };
            let (penetration, gap) = (sorted(|r| r.max(0.0)), sorted(|r| (-r).max(0.0)));
            let past = penetration.iter().filter(|&&r| r > 1.0).count();
            println!(
                "  {label}: {fine_reads} of {} read the fine grid | past the bar {past} | penetration over the bar \
                 p95 {:.3} p99 {:.3} p99.9 {:.3} worst {:.3} | shortfall p95 {:.3} worst {:.3} | band apart {apart:.1e} m",
                points.len(),
                quantile(&penetration, 0.95),
                quantile(&penetration, 0.99),
                quantile(&penetration, 0.999),
                quantile(&penetration, 1.0),
                quantile(&gap, 0.95),
                quantile(&gap, 1.0),
            );
            assert_eq!(
                fine_reads,
                points.len(),
                "every point on the scan reads the fine grid"
            );
            assert!(
                apart <= 1e-12,
                "the bake reads what the band reads: {apart:e} m apart"
            );
            assert_eq!(past, 0, "no point on the scan is past G2's bar");
            let worst = readings.iter().copied().fold(0.0_f64, f64::max);
            let from = (0..=10_u32).find(|&mm| g2_bar(f64::from(mm) * 1e-3) >= worst);
            println!("  {label}: meets G2's bar at insets from {from:?} mm");
            if product {
                assert!(
                    worst <= g2_bar(0.0),
                    "the product's fine grid meets G2's floor: {worst:e} m"
                );
            }
        }
    }
}

/// Where the welded scan's pseudo-normal sign fails away from the surface (§16u): its self-intersecting
/// triangle pairs, and the band samples at 0.25 mm where the pseudo-normal sign and the flood fill disagree a
/// quarter cell or more from the surface, each checked by parity and placed against the nearest intersecting
/// triangle.
#[test]
#[ignore = "needs the repo-excluded product scan; run with --release --ignored --nocapture"]
fn where_the_pseudo_normal_sign_fails_on_the_product_scan() {
    use mesh_repair::intersect::{IntersectionParams, detect_self_intersections};
    let (scan, _, _, _) = super::tests::product_scene().expect("the product scan must be present");
    // Weld exactly coincident vertices, as the bake does.
    let mut index: HashMap<[u64; 3], u32> = HashMap::new();
    let mut welded = IndexedMesh::new();
    let renumbered: Vec<u32> = scan
        .vertices
        .iter()
        .map(|p| {
            let key = [
                (p.x + 0.0).to_bits(),
                (p.y + 0.0).to_bits(),
                (p.z + 0.0).to_bits(),
            ];
            *index.entry(key).or_insert_with(|| {
                welded.vertices.push(*p);
                u32::try_from(welded.vertices.len() - 1).unwrap()
            })
        })
        .collect();
    welded.faces = scan
        .faces
        .iter()
        .map(|f| f.map(|v| renumbered[v as usize]))
        .collect();
    let crossing = detect_self_intersections(&welded, &IntersectionParams::exhaustive());
    println!(
        "\n══ where the pseudo-normal sign fails [LOCAL] ══\nwelded {} vertices, {} faces; self-intersecting pairs {} (truncated {})",
        welded.vertices.len(),
        welded.faces.len(),
        crossing.intersection_count,
        crossing.truncated
    );
    let mut crossed: Vec<u32> = crossing
        .intersecting_pairs
        .iter()
        .flat_map(|&(a, b)| [a, b])
        .collect();
    crossed.sort_unstable();
    crossed.dedup();
    let centroid = |f: u32| {
        let [a, b, c] = welded.faces[f as usize].map(|v| welded.vertices[v as usize].coords);
        Point3::from((a + b + c) / 3.0)
    };
    for &(a, b) in crossing.intersecting_pairs.iter().take(12) {
        println!(
            "  pair {a} × {b}: centroids {} and {}",
            centroid(a),
            centroid(b)
        );
    }

    let bounds = scan_aabb(&scan, 0.004);
    let cell = 0.000_25;
    let grid = layout(bounds, cell);
    let distance = TriMeshDistance::new(welded.clone()).unwrap();
    let pseudo = PseudoNormalSign::from_distance(&distance);
    let (flood, _) = FloodFillSign::build(&distance, bounds, cell, 0.75).unwrap();
    let parity = ParitySign::new(&welded).unwrap();
    let mut keys: Vec<[u32; 3]> = surface_points(&scan)
        .iter()
        .flat_map(|p| stencil(*p, grid).1)
        .collect();
    keys.sort_unstable();
    keys.dedup();
    let mut far = 0;
    for key in keys {
        let p = at(key, grid);
        let d = distance.distance(p);
        if d / cell < 0.25 || pseudo.is_inside(p) == flood.is_inside(p) {
            continue;
        }
        far += 1;
        let parity = parity.is_inside(p);
        let nearest = crossed
            .iter()
            .map(|&f| (centroid(f) - p).norm())
            .fold(f64::INFINITY, f64::min);
        if far <= 50 {
            println!(
                "  far dispute at {p}: {:.3} cells out; pseudo-normal inside {}, flood inside {}, parity inside {parity}; \
                 nearest intersecting triangle's centroid {:.4} mm away",
                d / cell,
                pseudo.is_inside(p),
                flood.is_inside(p),
                nearest * 1e3
            );
        }
    }
    println!("  far disputes: {far}");
}
