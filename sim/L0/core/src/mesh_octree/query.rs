//! A distance and its gradient read from a mesh's octree, as MuJoCo 3.5.0
//! reads them for a signed distance field geom with no plugin
//! (`boxProjection`, `findOct`, `oct_distance`, `oct_gradient`;
//! `engine_collision_sdf.c:35-200`), statement for statement.

use super::MeshOctree;

/// `findOct`'s slack on a node's box (`eps`).
const FIND_EPS: f64 = 1e-8;

/// `findOct`'s bound on its steps (`niter`).
const FIND_STEPS: usize = 100;

/// `boxProjection`'s step inside the box (`eps`).
const PROJECTION_EPS: f64 = 1e-6;

/// `oct_gradient`'s forward-difference step outside the root's box (`eps`).
const GRADIENT_EPS: f64 = 1e-8;

/// `mju_max`: `a` where `a >= b`, else `b`.
fn max(a: f64, b: f64) -> f64 {
    if a >= b { a } else { b }
}

/// `boxProjection` (`engine_collision_sdf.c:35-62`): for a point inside the
/// box (centre, half sizes) or on it, the largest of its distances past each
/// face pair (at most 0), the point kept; outside, its distance to the box,
/// the point moved along each axis it lies outside on to 1e-6 inside the
/// face.
fn box_projection(point: &mut [f64; 3], aabb: &[f64; 6]) -> f64 {
    let r = [point[0] - aabb[0], point[1] - aabb[1], point[2] - aabb[2]];
    let q = [
        r[0].abs() - aabb[3],
        r[1].abs() - aabb[4],
        r[2].abs() - aabb[5],
    ];
    let mut dist_sqr = 0.0;
    if q[0] <= 0.0 && q[1] <= 0.0 && q[2] <= 0.0 {
        return max(q[0], max(q[1], q[2]));
    }
    for i in 0..3 {
        if q[i] >= 0.0 {
            dist_sqr += q[i] * q[i];
            point[i] -= if r[i] > 0.0 {
                q[i] + PROJECTION_EPS
            } else {
                -(q[i] + PROJECTION_EPS)
            };
        }
    }
    dist_sqr.sqrt()
}

/// The leaf `findOct` stops on and the point's coordinates in it, each from
/// 0 at the box's low face to 1 at its high one.
struct Leaf {
    node: usize,
    coord: [f64; 3],
}

impl Leaf {
    /// The trilinear weight of each corner (`findOct`'s `w`), in the order of
    /// a node's corners: `x` high where `j & 1`, `y` where `j & 2`, `z` where
    /// `j & 4`.
    fn weights(&self) -> [f64; 8] {
        let c = self.coord;
        std::array::from_fn(|j| {
            (if j & 1 != 0 { c[0] } else { 1.0 - c[0] })
                * (if j & 2 != 0 { c[1] } else { 1.0 - c[1] })
                * (if j & 4 != 0 { c[2] } else { 1.0 - c[2] })
        })
    }

    /// Each weight's derivative in the leaf's coordinates (`findOct`'s `dw`),
    /// not divided by the leaf's size.
    fn weight_derivatives(&self) -> [[f64; 3]; 8] {
        let c = self.coord;
        let side = |j: usize, bit: usize, k: usize| if j & bit != 0 { c[k] } else { 1.0 - c[k] };
        let sign = |j: usize, bit: usize| if j & bit != 0 { 1.0 } else { -1.0 };
        std::array::from_fn(|j| {
            [
                sign(j, 1) * side(j, 2, 1) * side(j, 4, 2),
                side(j, 1, 0) * sign(j, 2) * side(j, 4, 2),
                side(j, 1, 0) * side(j, 2, 1) * sign(j, 4),
            ]
        })
    }
}

impl MeshOctree {
    /// `findOct` (`engine_collision_sdf.c:69-135`): from the root, each step
    /// the child whose half of its parent's box holds the point, until a
    /// leaf; a step on a node whose box (within 1e-8) does not hold the point
    /// is spent and the node tried again. `None` after 100 steps, where
    /// MuJoCo stops with "Node not found": a point outside the root's box.
    fn find(&self, p: &[f64; 3]) -> Option<Leaf> {
        let mut node = 0;
        for _ in 0..FIND_STEPS {
            let a = &self.aabb[node];
            let vmin = [a[0] - a[3], a[1] - a[4], a[2] - a[5]];
            let vmax = [a[0] + a[3], a[1] + a[4], a[2] + a[5]];
            if p[0] + FIND_EPS < vmin[0]
                || p[0] - FIND_EPS > vmax[0]
                || p[1] + FIND_EPS < vmin[1]
                || p[1] - FIND_EPS > vmax[1]
                || p[2] + FIND_EPS < vmin[2]
                || p[2] - FIND_EPS > vmax[2]
            {
                continue;
            }
            let coord = [
                (p[0] - vmin[0]) / (vmax[0] - vmin[0]),
                (p[1] - vmin[1]) / (vmax[1] - vmin[1]),
                (p[2] - vmin[2]) / (vmax[2] - vmin[2]),
            ];
            let child = &self.child[node];
            if child.iter().all(|&c| c == -1) {
                return Some(Leaf { node, coord });
            }
            // MuJoCo's ternary: a NaN coordinate takes the high child.
            #[allow(clippy::bool_to_int_with_if)]
            let slot = |k: usize| if coord[k] < 0.5 { 0 } else { 1 };
            let next = child[4 * slot(2) + 2 * slot(1) + slot(0)];
            // A node is split into all eight of its children
            // (`mjCOctree::Subdivide`), so a child of a node that has one is
            // never -1 (MuJoCo's "Invalid node number").
            node = usize::try_from(next).ok()?;
        }
        None
    }

    /// The signed distance at `point`, in the mesh's frame: MuJoCo's
    /// `oct_distance` (`engine_collision_sdf.c:138-158`). Inside the root's
    /// box, the leaf's corner coefficients weighted trilinearly; outside it,
    /// those at the point moved 1e-6 inside the box, plus the point's
    /// distance to the box.
    ///
    /// `None` where MuJoCo stops with "Node not found" (`findOct`): a point
    /// so far from the octree that its move inside the box rounds outside it
    /// again.
    #[must_use]
    pub fn distance(&self, point: [f64; 3]) -> Option<f64> {
        let mut point = point;
        let box_dist = box_projection(&mut point, &self.aabb[0]);
        let leaf = self.find(&point)?;
        let w = leaf.weights();
        let coeff = &self.coeff[leaf.node];
        let mut sdf = 0.0;
        for i in 0..8 {
            sdf += w[i] * coeff[i];
        }
        Some(if box_dist > 0.0 { sdf + box_dist } else { sdf })
    }

    /// The signed distance's gradient at `point`, in the mesh's frame:
    /// MuJoCo's `oct_gradient` (`engine_collision_sdf.c:162-200`). Inside
    /// the root's box (or on it), the derivative of the leaf's trilinear
    /// weights in the leaf's own coordinates, each from 0 to 1 across it, so
    /// the gradient's length scales with the leaf's size; outside it, forward
    /// differences of [`Self::distance`] with a step of 1e-8.
    ///
    /// `None` where [`Self::distance`] is.
    #[must_use]
    pub fn gradient(&self, point: [f64; 3]) -> Option<[f64; 3]> {
        let mut grad = [0.0; 3];
        let mut p = point;
        if box_projection(&mut p, &self.aabb[0]) <= 0.0 {
            let leaf = self.find(&p)?;
            let dw = leaf.weight_derivatives();
            let coeff = &self.coeff[leaf.node];
            for i in 0..8 {
                grad[0] += dw[i][0] * coeff[i];
                grad[1] += dw[i][1] * coeff[i];
                grad[2] += dw[i][2] * coeff[i];
            }
            return Some(grad);
        }
        let dist0 = self.distance(point)?;
        let dist_x = self.distance([point[0] + GRADIENT_EPS, point[1], point[2]])?;
        let dist_y = self.distance([point[0], point[1] + GRADIENT_EPS, point[2]])?;
        let dist_z = self.distance([point[0], point[1], point[2] + GRADIENT_EPS])?;
        grad[0] = (dist_x - dist0) / GRADIENT_EPS;
        grad[1] = (dist_y - dist0) / GRADIENT_EPS;
        grad[2] = (dist_z - dist0) / GRADIENT_EPS;
        Some(grad)
    }
}
