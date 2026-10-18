//! MuJoCo's midphase (`mj_collideTree`, `engine_collision_driver.c:622-847`):
//! the geom pairs of two bodies whose boxes overlap, found by descending the
//! two bodies' bounding-volume hierarchies together
//! ([`Model::compute_body_bvhs`]).
//!
//! An inner node's box is in its body's inertial frame (`xipos`, `ximat`), a
//! leaf's the geom's own (`geom_aabb` at `geom_xpos`, `geom_xmat`). Two boxes
//! overlap unless one of the six face axes separates them by more than the
//! margin (`mj_collideOBB`, `:524-618`). Not ported: the hierarchies of
//! flexes, whose collisions sim-core finds apart from the body pairs, and
//! `bvh_active`, which marks the nodes visited for drawing.

use nalgebra::{Matrix3, Vector3};

use super::assign_margin;
use super::broadphase::filter_sphere;
use crate::types::validation::MAX_VAL;
use crate::types::{Data, Model};

/// Append to `pairs` each pair of geoms, one of `body1`'s and one of
/// `body2`'s, whose leaves the midphase reaches and whose geoms pass the
/// bounding-sphere filter and overlap as boxes, in the order MuJoCo's
/// `mj_collideTree` reaches them. `false`, appending nothing, when either
/// body has no hierarchy.
pub(super) fn collide_tree(
    model: &Model,
    data: &Data,
    body1: usize,
    body2: usize,
    pairs: &mut Vec<(usize, usize)>,
) -> bool {
    // A model built by hand and never derived has no hierarchies.
    let hierarchy = |b: usize| model.body_bvhadr.get(b).copied().flatten();
    let (Some(adr1), Some(adr2)) = (hierarchy(body1), hierarchy(body2)) else {
        return false;
    };
    let tree1 = Hierarchy::of(model, adr1, model.body_bvhnum[body1]);
    let tree2 = Hierarchy::of(model, adr2, model.body_bvhnum[body2]);
    let frame1 = (&data.xipos[body1], &data.ximat[body1]);
    let frame2 = (&data.xipos[body2], &data.ximat[body2]);
    let mut axes: Option<Axes> = None;

    let mut stack = vec![(0, 0)];
    while let Some((node1, node2)) = stack.pop() {
        let (leaf1, leaf2) = (tree1.is_leaf(node1), tree2.is_leaf(node2));
        if leaf1 && leaf2 {
            // A leaf always holds its geom (`compute_body_bvhs`).
            let (Some(g1), Some(g2)) = (tree1.nodeid[node1], tree2.nodeid[node2]) else {
                continue;
            };
            let margin = assign_margin(model, model.geom_margin[g1] + model.geom_margin[g2]);
            if !filter_sphere(model, data, g1, g2, margin)
                && collide_obb(
                    [&model.geom_aabb[g1], &model.geom_aabb[g2]],
                    [
                        (&data.geom_xpos[g1], &data.geom_xmat[g1]),
                        (&data.geom_xpos[g2], &data.geom_xmat[g2]),
                    ],
                    margin,
                    None,
                )
            {
                pairs.push((g1, g2));
            }
            continue;
        }

        // Stop where the two inner boxes do not overlap.
        let margin = assign_margin(model, model.body_margin[body1] + model.body_margin[body2]);
        let axes = axes.get_or_insert_with(|| Axes::of([frame1, frame2]));
        if !collide_obb(
            [&tree1.aabb[node1], &tree2.aabb[node2]],
            [frame1, frame2],
            margin,
            Some(axes),
        ) {
            continue;
        }

        // Descend the hierarchy that is not at a leaf; with neither, the one
        // whose box has the larger surface (MuJoCo's surfaces, from each
        // box's half-sizes less its centre).
        let descend1 = if leaf1 || leaf2 {
            leaf2
        } else {
            surface(&tree1.aabb[node1]) > surface(&tree2.aabb[node2])
        };
        if descend1 {
            for child in tree1.child[node1].into_iter().flatten() {
                stack.push((child, node2));
            }
        } else {
            for child in tree2.child[node2].into_iter().flatten() {
                stack.push((node1, child));
            }
        }
    }
    true
}

/// One body's hierarchy: its nodes' boxes, children and geoms, each node
/// numbered from the body's root, 0.
struct Hierarchy<'a> {
    aabb: &'a [[f64; 6]],
    child: &'a [[Option<usize>; 2]],
    nodeid: &'a [Option<usize>],
}

impl<'a> Hierarchy<'a> {
    fn of(model: &'a Model, adr: usize, num: usize) -> Self {
        Self {
            aabb: &model.bvh_aabb[adr..adr + num],
            child: &model.bvh_child[adr..adr + num],
            nodeid: &model.bvh_nodeid[adr..adr + num],
        }
    }

    fn is_leaf(&self, node: usize) -> bool {
        self.child[node] == [None, None]
    }
}

/// The "surface" MuJoCo compares to choose which hierarchy to descend
/// (`engine_collision_driver.c:810-821`): its edges are each box's half-size
/// less its centre, as written there.
fn surface(aabb: &[f64; 6]) -> f64 {
    let (x, y, z) = (aabb[3] - aabb[0], aabb[4] - aabb[1], aabb[5] - aabb[2]);
    x * y + y * z + z * x
}

/// For two frames that stay put, the dot products `mj_collideOBB` reads at
/// every inner node, computed once (`product`, `offset`,
/// `engine_collision_driver.c:572-585`): `product[i][j][k][l]` is frame `i`'s
/// axis `l` along frame `j`'s axis `k`, `offset[i][j][k]` frame `i`'s
/// position along frame `j`'s axis `k`.
struct Axes {
    product: [[[[f64; 3]; 3]; 2]; 2],
    offset: [[[f64; 3]; 2]; 2],
}

impl Axes {
    fn of(frames: [(&Vector3<f64>, &Matrix3<f64>); 2]) -> Self {
        let normal = |i: usize, j: usize| -> [f64; 3] {
            let m = frames[i].1;
            [m[(0, j)], m[(1, j)], m[(2, j)]]
        };
        let mut product = [[[[0.0; 3]; 3]; 2]; 2];
        let mut offset = [[[0.0; 3]; 2]; 2];
        for i in 0..2 {
            for j in 0..2 {
                for k in 0..3 {
                    product[i][j][k] = std::array::from_fn(|l| dot(&normal(i, l), &normal(j, k)));
                    let p = frames[i].0;
                    offset[i][j][k] = dot(&[p.x, p.y, p.z], &normal(j, k));
                }
            }
        }
        Self { product, offset }
    }
}

/// `mju_dot3`.
fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Whether two boxes overlap within `margin` (`mj_collideOBB`,
/// `engine_collision_driver.c:524-618`): each box `[cx, cy, cz, hx, hy, hz]`
/// in its frame (a position and a turn), tested along the three face axes of
/// each (Gottschalk et al.), from `axes` when given. A box as wide as
/// [`MAX_VAL`] along all three axes overlaps everything; along one or two,
/// only its own axes are tested.
fn collide_obb(
    aabb: [&[f64; 6]; 2],
    frames: [(&Vector3<f64>, &Matrix3<f64>); 2],
    margin: f64,
    axes: Option<&Axes>,
) -> bool {
    let wide = |b: &[f64; 6]| [b[3] >= MAX_VAL, b[4] >= MAX_VAL, b[5] >= MAX_VAL];
    let wide = [wide(aabb[0]), wide(aabb[1])];
    if wide.iter().any(|w| w.iter().all(|&x| x)) {
        return true;
    }
    let infinite = [wide[0].iter().any(|&x| x), wide[1].iter().any(|&x| x)];
    let normal = |i: usize, j: usize| -> [f64; 3] {
        let m = frames[i].1;
        [m[(0, j)], m[(1, j)], m[(2, j)]]
    };
    let center = |i: usize| -> [f64; 3] {
        let (p, m, b) = (frames[i].0, frames[i].1, aabb[i]);
        [
            m[(0, 0)] * b[0] + m[(0, 1)] * b[1] + m[(0, 2)] * b[2] + p.x,
            m[(1, 0)] * b[0] + m[(1, 1)] * b[1] + m[(1, 2)] * b[2] + p.y,
            m[(2, 0)] * b[0] + m[(2, 1)] * b[1] + m[(2, 2)] * b[2] + p.z,
        ]
    };
    for j in 0..2 {
        // Skip the axes of a box tested against an infinite one.
        if infinite[1 - j] {
            continue;
        }
        for k in 0..3 {
            let mut proj = [0.0; 2];
            let mut radius = [0.0; 2];
            for i in 0..2 {
                let b = aabb[i];
                if let Some(axes) = axes {
                    let p = &axes.product[i][j][k];
                    proj[i] = b[0] * p[0] + b[1] * p[1] + b[2] * p[2] + axes.offset[i][j][k];
                    radius[i] = (b[3] * p[0]).abs() + (b[4] * p[1]).abs() + (b[5] * p[2]).abs();
                } else {
                    let n = normal(j, k);
                    proj[i] = dot(&center(i), &n);
                    radius[i] = (b[3] * dot(&normal(i, 0), &n)).abs()
                        + (b[4] * dot(&normal(i, 1), &n)).abs()
                        + (b[5] * dot(&normal(i, 2), &n)).abs();
                }
            }
            if radius[0] + radius[1] + margin < (proj[1] - proj[0]).abs() {
                return false;
            }
        }
    }
    true
}
