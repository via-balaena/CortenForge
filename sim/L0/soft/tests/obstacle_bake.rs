//! The explicit solver's obstacle bake (`sim_soft::obstacle`, plan §16u): a box's own distance at every sample,
//! square to the grid and turned off it, whichever way it is wound; the fine grid wherever a point within the band
//! looks; a triangle soup welded; and a mesh that is not closed, or not a mesh, refused.
//!
//! The mesh's distance is computed in f32 (parry's scalar), so it differs from the box's by f32's roundings on the
//! box's size; 1e-6 of the size bounds them loosely.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]

use mesh_types::IndexedMesh;
use nalgebra::{Point3, Unit, UnitQuaternion, Vector3};
use sim_soft::obstacle::{BakedObstacle, ObstacleBake, ObstacleBakeError, bake_obstacle};
use sim_soft_explicit::executor::{Obstacle, check_obstacle};
use sim_soft_explicit::f64::{Pose, SDF_BRICK, SDF_NO_BRICK, SdfGridLayout};

/// A box: `low` to `high` in its own frame, turned by `turn` about the origin into the mesh's.
#[derive(Clone, Copy)]
struct Cuboid {
    low: [f64; 3],
    high: [f64; 3],
    turn: UnitQuaternion<f64>,
}

impl Cuboid {
    /// As a triangle soup: three vertices a face, as an STL loads, wound outward.
    fn soup(self) -> IndexedMesh {
        let corner = |i: usize| {
            self.turn
                * Point3::new(
                    if i & 1 == 0 {
                        self.low[0]
                    } else {
                        self.high[0]
                    },
                    if i & 2 == 0 {
                        self.low[1]
                    } else {
                        self.high[1]
                    },
                    if i & 4 == 0 {
                        self.low[2]
                    } else {
                        self.high[2]
                    },
                )
        };
        let faces = [
            [0, 2, 1],
            [1, 2, 3],
            [4, 5, 6],
            [5, 7, 6],
            [0, 1, 4],
            [1, 5, 4],
            [2, 6, 3],
            [3, 6, 7],
            [0, 4, 2],
            [2, 4, 6],
            [1, 3, 5],
            [3, 7, 5],
        ];
        let mut soup = IndexedMesh::new();
        for face in faces {
            let first = u32::try_from(soup.vertices.len()).unwrap();
            for corner_index in face {
                soup.vertices.push(corner(corner_index));
            }
            soup.faces.push([first, first + 1, first + 2]);
        }
        soup
    }

    /// The exact signed distance to the box.
    fn distance(self, p: Point3<f64>) -> f64 {
        let p = self.turn.inverse() * p;
        let q: [f64; 3] = std::array::from_fn(|a| {
            let (centre, half) = (
                f64::midpoint(self.low[a], self.high[a]),
                (self.high[a] - self.low[a]) / 2.0,
            );
            (p[a] - centre).abs() - half
        });
        let outside = q.iter().map(|x| x.max(0.0).powi(2)).sum::<f64>().sqrt();
        outside + q[0].max(q[1]).max(q[2]).min(0.0)
    }
}

const SQUARE: Cuboid = Cuboid {
    low: [-0.012, 0.003, -0.02],
    high: [0.009, 0.017, 0.011],
    turn: UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(1.0, 0.0, 0.0, 0.0)),
};

/// The same box turned off every axis of the grid.
fn turned() -> Cuboid {
    Cuboid {
        turn: UnitQuaternion::from_axis_angle(
            &Unit::new_normalize(Vector3::new(1.0, 2.0, 3.0)),
            0.7,
        ),
        ..SQUARE
    }
}

const BAKE: ObstacleBake = ObstacleBake {
    coarse_cell: 0.002,
    fine_cell: 0.0005,
    band: 0.0005,
    margin: 0.003,
};
/// f32's roundings on the box's size, loosely: 1e-6 of its longest side.
const MATCH: f64 = 1e-6 * 0.031;

fn at(grid: SdfGridLayout, i: u32, j: u32, k: u32) -> Point3<f64> {
    Point3::new(
        grid.origin_x + f64::from(i) * grid.cell_size,
        grid.origin_y + f64::from(j) * grid.cell_size,
        grid.origin_z + f64::from(k) * grid.cell_size,
    )
}

/// Each brick of the fine lattice (kept or not): its map index's slot, and its samples' positions in storage
/// order. Brick (bx, by, bz)'s sample (x, y, z) is at (bx·8 + x, by·8 + y, bz·8 + z), x fastest.
fn bricks(baked: &BakedObstacle) -> Vec<(u32, Vec<Point3<f64>>)> {
    let f = baked.fine.grid;
    let count = |size: u32| size.div_ceil(SDF_BRICK);
    let (bx, by) = (count(f.size_x), count(f.size_y));
    let mut out = Vec::new();
    for (index, &slot) in baked.fine.map.iter().enumerate() {
        let index = u32::try_from(index).unwrap();
        let brick = [index % bx, index / bx % by, index / (bx * by)];
        let mut points = Vec::new();
        for z in 0..SDF_BRICK {
            for y in 0..SDF_BRICK {
                for x in 0..SDF_BRICK {
                    points.push(at(
                        f,
                        brick[0] * SDF_BRICK + x,
                        brick[1] * SDF_BRICK + y,
                        brick[2] * SDF_BRICK + z,
                    ));
                }
            }
        }
        out.push((slot, points));
    }
    out
}

/// Every sample of both grids against the box's own distance.
fn assert_every_sample_is_the_boxs(cuboid: Cuboid, baked: &BakedObstacle) {
    let g = baked.grid;
    let mut n = 0;
    for k in 0..g.size_z {
        for j in 0..g.size_y {
            for i in 0..g.size_x {
                let p = at(g, i, j, k);
                let exact = cuboid.distance(p);
                assert!(
                    (baked.values[n] - exact).abs() < MATCH,
                    "{p}: {} {exact}",
                    baked.values[n]
                );
                n += 1;
            }
        }
    }
    let per_brick = SDF_BRICK.pow(3) as usize;
    let mut kept = 0;
    for (slot, points) in bricks(baked) {
        if slot == SDF_NO_BRICK {
            continue;
        }
        kept += 1;
        for (within, p) in points.into_iter().enumerate() {
            let value = baked.fine.values[slot as usize * per_brick + within];
            let exact = cuboid.distance(p);
            assert!((value - exact).abs() < MATCH, "{p}: {value} {exact}");
        }
    }
    assert!(kept > 0 && kept < baked.fine.map.len());
}

#[test]
fn a_box_bakes_to_its_own_distance_at_every_sample() {
    for cuboid in [SQUARE, turned()] {
        let baked = bake_obstacle(&cuboid.soup(), BAKE).unwrap();
        assert_every_sample_is_the_boxs(cuboid, &baked);
    }
}

#[test]
fn a_box_wound_inward_bakes_to_the_same_grids() {
    // The sign is a ray's crossings, which do not care which way a face is wound.
    let cuboid = turned();
    let outward = bake_obstacle(&cuboid.soup(), BAKE).unwrap();
    let mut inward = cuboid.soup();
    for face in &mut inward.faces {
        face.swap(1, 2);
    }
    let inward = bake_obstacle(&inward, BAKE).unwrap();
    assert_eq!(inward.grid, outward.grid);
    assert_eq!(inward.fine.grid, outward.fine.grid);
    assert_eq!(inward.fine.map, outward.fine.map);
    for (a, b) in [
        (&inward.values, &outward.values),
        (&inward.fine.values, &outward.fine.values),
    ] {
        assert_eq!(a.len(), b.len());
        for (a, b) in a.iter().zip(b) {
            assert!((a - b).abs() < MATCH, "{a} {b}");
        }
    }
    assert_every_sample_is_the_boxs(cuboid, &inward);
}

#[test]
fn a_soup_with_signed_zeros_welds_closed() {
    // A box with a face on the plane y = 0, half its copies of that plane's corners at −0.0: they are one vertex
    // each, or the box is open at that face's edges.
    let cuboid = Cuboid {
        low: [-0.012, 0.0, -0.02],
        ..SQUARE
    };
    let mut soup = cuboid.soup();
    let mut flipped = 0;
    for (n, v) in soup.vertices.iter_mut().enumerate() {
        if v.y == 0.0 && n % 2 == 0 {
            v.y = -0.0;
            flipped += 1;
        }
    }
    assert!(flipped > 0);
    let baked = bake_obstacle(&soup, BAKE).unwrap();
    assert_every_sample_is_the_boxs(cuboid, &baked);
}

#[test]
fn every_point_within_the_band_reads_the_fine_grid() {
    // Points on the surface and within the band of it, both sides, all over the box, edges and corners included.
    let cuboid = turned();
    let baked = bake_obstacle(&cuboid.soup(), BAKE).unwrap();
    let (low, high) = (cuboid.low, cuboid.high);
    let mut points = 0;
    for n in 0..4000_u32 {
        let t = f64::from(n);
        let u: [f64; 3] =
            std::array::from_fn(|a| (t * [0.618_034, 0.414_214, 0.732_051][a]).fract());
        // A point on the box's surface: clamp a point in the box onto a face.
        let face = n % 6;
        let mut p: [f64; 3] = std::array::from_fn(|a| low[a] + u[a] * (high[a] - low[a]));
        let axis = (face / 2) as usize;
        p[axis] = if face % 2 == 0 { low[axis] } else { high[axis] };
        // Then off it by up to the band, out or in, along the face's normal and across it.
        let off = (2.0 * (t * 0.271_828).fract() - 1.0) * BAKE.band;
        let normal = if face % 2 == 0 { -1.0 } else { 1.0 };
        p[axis] += normal * off;
        p[(axis + 1) % 3] += 0.3 * off;
        let point = cuboid.turn * Point3::from(p);
        if cuboid.distance(point).abs() <= BAKE.band {
            assert!(baked.fine.sample(point.into()).is_some(), "{point}");
            points += 1;
        }
    }
    assert!(points > 3000, "{points}");
}

#[test]
fn a_brick_is_kept_exactly_when_a_sample_is_near_the_surface() {
    // Near: within the band and 2√3 fine cells, the farthest a lookup from a point within the band reaches. Every
    // brick of the lattice, by the box's exact distance at its samples (up to f32's rounding at the threshold);
    // the box turned, so the surface crosses bricks at every angle.
    let cuboid = turned();
    let baked = bake_obstacle(&cuboid.soup(), BAKE).unwrap();
    let near = BAKE.band + 2.0 * 3.0_f64.sqrt() * BAKE.fine_cell;
    let (mut kept, mut dropped) = (0, 0);
    for (slot, points) in bricks(&baked) {
        let nearest = points
            .iter()
            .map(|&p| cuboid.distance(p).abs())
            .fold(f64::INFINITY, f64::min);
        if (nearest - near).abs() < MATCH {
            continue;
        }
        assert_eq!(slot != SDF_NO_BRICK, nearest <= near, "nearest {nearest}");
        if slot == SDF_NO_BRICK {
            dropped += 1;
        } else {
            kept += 1;
        }
    }
    assert!(kept > 0 && dropped > 0, "{kept} {dropped}");
}

#[test]
fn a_fine_lattice_is_counted_by_its_bricks() {
    // A box a tenth of a millimetre across, in a margin that makes the fine lattice longer than a u32 counts in
    // samples, though not in bricks: it bakes, and its surface reads the fine grid.
    let tiny = Cuboid {
        low: [0.0; 3],
        high: [1e-4; 3],
        ..SQUARE
    };
    let bake = ObstacleBake {
        coarse_cell: 5e-4,
        fine_cell: 1e-5,
        band: 1e-5,
        margin: 0.008_45,
    };
    let baked = bake_obstacle(&tiny.soup(), bake).unwrap();
    let f = baked.fine.grid;
    assert!(u64::from(f.size_x) * u64::from(f.size_y) * u64::from(f.size_z) > u64::from(u32::MAX));
    for p in [[5e-5, 5e-5, 0.0], [1e-4, 3e-5, 7e-5], [0.0, 0.0, 0.0]] {
        let sample = baked
            .fine
            .sample(p)
            .expect("the surface reads the fine grid");
        assert!(sample.distance.abs() < MATCH, "{p:?}: {}", sample.distance);
    }
}

#[test]
fn the_baked_obstacle_is_a_valid_one() {
    let baked = bake_obstacle(&turned().soup(), BAKE).unwrap();
    let obstacle = Obstacle {
        grid: baked.grid,
        values: baked.values,
        fine: Some(baked.fine),
        start: 0.0,
        interval: 1.0,
        poses: vec![Pose {
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
    assert_eq!(check_obstacle(&obstacle), Ok(()));
}

#[test]
fn a_mesh_that_is_not_closed_is_refused() {
    // A face removed: its two box edges and its diagonal each have one face left.
    let mut open = SQUARE.soup();
    open.faces.pop();
    assert_eq!(
        bake_obstacle(&open, BAKE).unwrap_err(),
        ObstacleBakeError::Open { edges: 3 }
    );
    // A square with a face on each side: every edge has two faces but the diagonal, which has four.
    let mut sheet = IndexedMesh::new();
    for (x, y) in [(0.0, 0.0), (0.01, 0.0), (0.01, 0.01), (0.0, 0.01)] {
        sheet.vertices.push(Point3::new(x, y, 0.0));
    }
    sheet.faces = vec![[0, 1, 2], [0, 2, 3], [0, 2, 1], [0, 3, 2]];
    assert_eq!(
        bake_obstacle(&sheet, BAKE).unwrap_err(),
        ObstacleBakeError::Open { edges: 1 }
    );
}

#[test]
fn a_broken_mesh_or_bake_is_refused() {
    assert_eq!(
        bake_obstacle(&IndexedMesh::new(), BAKE).unwrap_err(),
        ObstacleBakeError::Mesh
    );
    let mut missing = SQUARE.soup();
    let count = u32::try_from(missing.vertices.len()).unwrap();
    missing.faces[0][2] = count;
    assert_eq!(
        bake_obstacle(&missing, BAKE).unwrap_err(),
        ObstacleBakeError::Mesh
    );
    let mut infinite = SQUARE.soup();
    infinite.vertices[0].z = f64::INFINITY;
    assert_eq!(
        bake_obstacle(&infinite, BAKE).unwrap_err(),
        ObstacleBakeError::Mesh
    );
    // A tetrahedron's faces on four points in a line: closed, and no triangle has area.
    let mut flat = IndexedMesh::new();
    for x in [0.0, 0.01, 0.02, 0.03] {
        flat.vertices.push(Point3::new(x, 0.0, 0.0));
    }
    flat.faces = vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]];
    assert_eq!(
        bake_obstacle(&flat, BAKE).unwrap_err(),
        ObstacleBakeError::Mesh
    );

    let square = SQUARE.soup();
    for bad in [
        ObstacleBake { band: 0.0, ..BAKE },
        ObstacleBake {
            coarse_cell: f64::NAN,
            ..BAKE
        },
        ObstacleBake {
            fine_cell: -BAKE.fine_cell,
            ..BAKE
        },
        ObstacleBake {
            margin: f64::INFINITY,
            ..BAKE
        },
        // Wider than the margin, so the grids would end inside the band.
        ObstacleBake {
            band: BAKE.margin * 1.01,
            ..BAKE
        },
    ] {
        assert_eq!(
            bake_obstacle(&square, bad).unwrap_err(),
            ObstacleBakeError::Bake,
            "{bad:?}"
        );
    }
    // Grids too fine to count: the coarse grid's samples (though not its bricks, were it bricked), the fine
    // lattice's bricks, and a lattice whose sides' product passes a u64.
    for bad in [
        ObstacleBake {
            coarse_cell: 1e-5,
            ..BAKE
        },
        ObstacleBake {
            fine_cell: 1e-8,
            ..BAKE
        },
        ObstacleBake {
            coarse_cell: 1e-11,
            ..BAKE
        },
    ] {
        assert_eq!(
            bake_obstacle(&square, bad).unwrap_err(),
            ObstacleBakeError::TooLarge,
            "{bad:?}"
        );
    }
}
