//! MuJoCo's midphase (`mj_collideTree`, `engine_collision_driver.c:622-847`):
//! the geom pairs of two bodies whose boxes overlap, found by descending the
//! two bodies' bounding-volume hierarchies together
//! ([`Model::compute_body_bvhs`]).
//!
//! An inner node's box is in its body's inertial frame (`xipos`, `ximat`), a
//! leaf's the geom's own (`geom_aabb` at `geom_xpos`, `geom_xmat`). Two boxes
//! overlap unless one of the six face axes separates them by more than the
//! margin (`mj_collideOBB`, `:524-618`). A body's hierarchy also descends
//! with a flex's ([`collide_tree_flex`]), whose boxes are its elements'
//! each step, in world coordinates (`bvh_aabb_dyn`). Not ported: two flexes'
//! hierarchies, whose collisions sim-core finds apart from the pairs, and
//! `bvh_active`, which marks the nodes visited for drawing.

use nalgebra::{Matrix3, Vector3};

use super::assign_margin;
use super::broadphase::{filter_sphere, rbound};
use super::flex_rigid::filter_bitmask;
use crate::types::validation::MAX_VAL;
use crate::types::{Data, GeomType, Model};

/// A box's frame: the position and turn of its body or geom, or none for a
/// box already in world coordinates (a flex's; MuJoCo's NULL frame).
type BoxFrame<'a> = Option<(&'a Vector3<f64>, &'a Matrix3<f64>)>;

/// Append to `pairs` each pair of geoms, one of `body1`'s and one of
/// `body2`'s, whose leaves the midphase reaches and whose geoms pass the
/// bounding-sphere filter and overlap as boxes, in the order MuJoCo's
/// `mj_collideTree` reaches them. `false`, appending nothing, when either
/// body has no hierarchy. `stack` is scratch for the descent, reused across
/// calls (MuJoCo's comes from its arena, `engine_collision_driver.c:649-653`).
pub(super) fn collide_tree(
    model: &Model,
    data: &Data,
    body1: usize,
    body2: usize,
    pairs: &mut Vec<(usize, usize)>,
    stack: &mut Vec<(usize, usize)>,
) -> bool {
    // A model built by hand and never derived has no hierarchies.
    let hierarchy = |b: usize| model.body_bvhadr.get(b).copied().flatten();
    let (Some(adr1), Some(adr2)) = (hierarchy(body1), hierarchy(body2)) else {
        return false;
    };
    let tree1 = Hierarchy::of(model, adr1, model.body_bvhnum[body1]);
    let tree2 = Hierarchy::of(model, adr2, model.body_bvhnum[body2]);
    let frame1 = Some((&data.xipos[body1], &data.ximat[body1]));
    let frame2 = Some((&data.xipos[body2], &data.ximat[body2]));
    let mut axes: Option<Axes> = None;

    stack.clear();
    stack.push((0, 0));
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
                        Some((&data.geom_xpos[g1], &data.geom_xmat[g1])),
                        Some((&data.geom_xpos[g2], &data.geom_xmat[g2])),
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
        descend(stack, (&tree1, node1, leaf1), (&tree2, node2, leaf2));
    }
    true
}

/// Append to `hits` each pair of a geom of `body` and an element of flex
/// `flex` whose leaves the midphase reaches, in the order MuJoCo's
/// `mj_collideTree` reaches them (`engine_collision_driver.c:720-754`): the
/// geom's bitmask meets the flex's, its bounding sphere grown by the margin
/// meets the element's box, and its box the element's (in world
/// coordinates); a plane or a signed distance field is left out (the caller
/// takes their vertices). Inner
/// boxes are tested within the body's and the flex's margins. `false`,
/// appending nothing, when either has no hierarchy.
pub(super) fn collide_tree_flex(
    model: &Model,
    data: &Data,
    body: usize,
    flex: usize,
    hits: &mut Vec<(usize, usize)>,
    stack: &mut Vec<(usize, usize)>,
) -> bool {
    let body_tree = model.body_bvhadr.get(body).copied().flatten();
    let flex_tree = model.flex_bvhadr.get(flex).copied().flatten();
    let (Some(adr1), Some(adr2)) = (body_tree, flex_tree) else {
        return false;
    };
    let tree1 = Hierarchy::of(model, adr1, model.body_bvhnum[body]);
    let tree2 = Hierarchy::of_flex(model, data, adr2, model.flex_bvhnum[flex]);
    let frame1 = Some((&data.xipos[body], &data.ximat[body]));
    let mut axes: Option<Axes> = None;

    stack.clear();
    stack.push((0, 0));
    while let Some((node1, node2)) = stack.pop() {
        let (leaf1, leaf2) = (tree1.is_leaf(node1), tree2.is_leaf(node2));
        if leaf1 && leaf2 {
            // A leaf always holds its geom or element.
            let (Some(g), Some(e)) = (tree1.nodeid[node1], tree2.nodeid[node2]) else {
                continue;
            };
            let margin = assign_margin(model, model.geom_margin[g] + model.flex_margin[flex]);
            if !filter_bitmask(model, g, flex)
                && !filter_sphere_box(
                    &data.geom_xpos[g],
                    rbound(model, g) + margin,
                    &tree2.aabb[node2],
                )
                && collide_obb(
                    [&model.geom_aabb[g], &tree2.aabb[node2]],
                    [Some((&data.geom_xpos[g], &data.geom_xmat[g])), None],
                    margin,
                    None,
                )
                && !matches!(model.geom_type[g], GeomType::Plane | GeomType::Sdf)
            {
                hits.push((g, e));
            }
            continue;
        }

        // Stop where the two inner boxes do not overlap.
        let margin = assign_margin(model, model.body_margin[body] + model.flex_margin[flex]);
        let axes = axes.get_or_insert_with(|| Axes::of([frame1, None]));
        if !collide_obb(
            [&tree1.aabb[node1], &tree2.aabb[node2]],
            [frame1, None],
            margin,
            Some(axes),
        ) {
            continue;
        }
        descend(stack, (&tree1, node1, leaf1), (&tree2, node2, leaf2));
    }
    true
}

/// Push the next node pairs: descend the hierarchy that is not at a leaf;
/// with neither, the one whose box has the larger surface (MuJoCo's
/// surfaces, from each box's half-sizes less its centre).
fn descend(
    stack: &mut Vec<(usize, usize)>,
    (tree1, node1, leaf1): (&Hierarchy<'_>, usize, bool),
    (tree2, node2, leaf2): (&Hierarchy<'_>, usize, bool),
) {
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

/// `filterSphereBox` (`engine_collision_driver.c:118-126`): whether a sphere
/// at `s` of radius `bound`, taken as its box, is clear of `aabb` (centre
/// and half-sizes) along an axis.
pub(super) fn filter_sphere_box(s: &Vector3<f64>, bound: f64, aabb: &[f64; 6]) -> bool {
    s.x + bound < aabb[0] - aabb[3]
        || s.y + bound < aabb[1] - aabb[4]
        || s.z + bound < aabb[2] - aabb[5]
        || s.x - bound > aabb[0] + aabb[3]
        || s.y - bound > aabb[1] + aabb[4]
        || s.z - bound > aabb[2] + aabb[5]
}

/// One body's or flex's hierarchy: its nodes' boxes, children and geoms or
/// elements, each node numbered from its root, 0.
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

    /// A flex's: its boxes this step's (`bvh_aabb_dyn`, numbered after the
    /// static nodes), its children and elements the model's.
    fn of_flex(model: &'a Model, data: &'a Data, adr: usize, num: usize) -> Self {
        let dynamic = adr - model.nbvhstatic;
        Self {
            aabb: &data.bvh_aabb_dyn[dynamic..dynamic + num],
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
    fn of(frames: [BoxFrame<'_>; 2]) -> Self {
        let normal = |i: usize, j: usize| face_axis(frames[i], j);
        let mut product = [[[[0.0; 3]; 3]; 2]; 2];
        let mut offset = [[[0.0; 3]; 2]; 2];
        for i in 0..2 {
            for j in 0..2 {
                for k in 0..3 {
                    product[i][j][k] = std::array::from_fn(|l| dot(&normal(i, l), &normal(j, k)));
                    offset[i][j][k] =
                        frames[i].map_or(0.0, |(p, _)| dot(&[p.x, p.y, p.z], &normal(j, k)));
                }
            }
        }
        Self { product, offset }
    }
}

/// Face axis `j` of a box in `frame`, in world coordinates: its turn's
/// column `j`, or the world's axis `j` without a frame.
fn face_axis(frame: BoxFrame<'_>, j: usize) -> [f64; 3] {
    match frame {
        Some((_, m)) => [m[(0, j)], m[(1, j)], m[(2, j)]],
        None => std::array::from_fn(|k| if j == k { 1.0 } else { 0.0 }),
    }
}

/// `mju_dot3`.
fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Whether two boxes overlap within `margin` (`mj_collideOBB`,
/// `engine_collision_driver.c:524-618`): each box `[cx, cy, cz, hx, hy, hz]`
/// in its frame (a position and a turn, or none), tested along the three face axes of
/// each (Gottschalk et al.), from `axes` when given. A box as wide as
/// [`MAX_VAL`] along all three axes overlaps everything; along one or two,
/// only its own axes are tested.
fn collide_obb(
    aabb: [&[f64; 6]; 2],
    frames: [BoxFrame<'_>; 2],
    margin: f64,
    axes: Option<&Axes>,
) -> bool {
    let wide = |b: &[f64; 6]| [b[3] >= MAX_VAL, b[4] >= MAX_VAL, b[5] >= MAX_VAL];
    let wide = [wide(aabb[0]), wide(aabb[1])];
    if wide.iter().any(|w| w.iter().all(|&x| x)) {
        return true;
    }
    let infinite = [wide[0].iter().any(|&x| x), wide[1].iter().any(|&x| x)];
    let normal = |i: usize, j: usize| face_axis(frames[i], j);
    let center = |i: usize| -> [f64; 3] {
        let b = aabb[i];
        match frames[i] {
            Some((p, m)) => [
                m[(0, 0)] * b[0] + m[(0, 1)] * b[1] + m[(0, 2)] * b[2] + p.x,
                m[(1, 0)] * b[0] + m[(1, 1)] * b[1] + m[(1, 2)] * b[2] + p.y,
                m[(2, 0)] * b[0] + m[(2, 1)] * b[1] + m[(2, 2)] * b[2] + p.z,
            ],
            None => [b[0], b[1], b[2]],
        }
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
