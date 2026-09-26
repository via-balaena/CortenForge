//! Plan §16b's K6: a rigid cylinder pressed into an elastic block, then
//! pushed sideways with less than `μ_f P` and back, in plane strain.
//!
//! Part of the contact sticks and part slips, and the stick zone's half-width
//! is judged against Cattaneo–Mindlin's closed form while the load rises and
//! Mindlin–Deresiewicz's while it falls.
//!
//! Lengths are in metres. The block's top is `z = 0`, its width runs along
//! `x`, and it is one element thick in `y`, with every node held in `y`. The
//! cylinder's axis runs along `y`; it is pressed along `−z` and pushed along
//! `+x`. The world's and the cylinder's body frame's origins are both at the
//! first point of contact (plan §16b: the reason is precision).

use std::f64::consts::TAU;

use super::grid::{BakeError, bake};
use crate::executor::{Executor, Obstacle, ObstacleError, Snapshot};
use crate::f64::{
    Material, Pose, SdfGridLayout, pose_interpolate, pose_sample_span, pose_unrotate, tet4_volume,
};
use crate::stepping::{RunError, Stepper, StepperConfig, gates};
use crate::{ExplicitModel, ModelError};

/// The stick zone's half-width over `a` while the tangential load rises, at
/// load fraction `Q/(μ_f P)`: Cattaneo–Mindlin's `√(1 − Q/(μ_f P))`
/// (Lorez & Pundir, arXiv 2412.14972, eq. 33).
#[must_use]
pub fn stick_while_loading(fraction: f64) -> f64 {
    (1.0 - fraction).max(0.0).sqrt()
}

/// The half-width over `a` of the zone that has not slipped back, at
/// fraction `ΔQ/(μ_f P)` after the tangential load falls from its peak by
/// `ΔQ`.
///
/// Mindlin–Deresiewicz's `√(1 − ΔQ/(2 μ_f P))`, in the half-plane form of
/// Andresen & Hills, arXiv 1911.07789, eqs. 10–12.
#[must_use]
pub fn stick_while_unloading(fraction: f64) -> f64 {
    (1.0 - 0.5 * fraction).max(0.0).sqrt()
}

/// Plan §16b's block: 20a wide and 10a deep, with elements of size `h` over
/// `|x| ≤ 1.5a` and the top 0.5a, graded to about a/2 at the far boundaries.
///
/// Each cell is a brick split into six tetrahedra around the same body
/// diagonal (the Kuhn split, as the tube's), so shared faces match. The one
/// layer of bricks is `h` thick, so the contact's cells are cubes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Block {
    /// The contact half-width `a` the press stops at.
    pub contact_half_width: f64,
    /// Half the block's width.
    pub half_width: f64,
    /// The block's depth.
    pub depth: f64,
    /// The element size `h` of the fine region, and the block's thickness.
    pub fine: f64,
    /// The fine region's half-width.
    pub fine_half_width: f64,
    /// The fine region's depth below the top.
    pub fine_depth: f64,
    /// The element size the grading reaches at the far boundaries.
    pub coarsest: f64,
}

impl Block {
    /// Plan §16b's block for contact half-width `a`, with `a/h = divisions`.
    #[must_use]
    pub fn plan(a: f64, divisions: f64) -> Self {
        Self {
            contact_half_width: a,
            half_width: 10.0 * a,
            depth: 10.0 * a,
            fine: a / divisions,
            fine_half_width: 1.5 * a,
            fine_depth: 0.5 * a,
            coarsest: 0.5 * a,
        }
    }

    /// The x of each column of nodes, ascending, symmetric about 0.
    #[must_use]
    pub fn columns(&self) -> Vec<f64> {
        let half = graded(
            self.fine,
            self.fine_half_width,
            self.half_width,
            self.coarsest,
        );
        half.iter()
            .rev()
            .map(|&x| -x)
            .chain(half.iter().skip(1).copied())
            .collect()
    }

    /// The z of each layer of nodes, from the top (0) down to `−depth`.
    #[must_use]
    pub fn layers(&self) -> Vec<f64> {
        graded(self.fine, self.fine_depth, self.depth, self.coarsest)
            .iter()
            .map(|&z| -z)
            .collect()
    }

    /// The node at column `i`, row `j` (0 at `y = 0`, 1 at `y = h`) and layer
    /// `k`, for a block with `columns` columns.
    // Truncation needs over 4 billion nodes; `model` refuses such a block.
    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub const fn node(columns: usize, i: usize, j: usize, k: usize) -> u32 {
        ((k * 2 + j) * columns + i) as u32
    }

    /// Every node's rest position.
    #[must_use]
    pub fn rest_positions(&self) -> Vec<[f64; 3]> {
        let (columns, layers) = (self.columns(), self.layers());
        let mut positions = Vec::with_capacity(columns.len() * 2 * layers.len());
        for &z in &layers {
            for y in [0.0, self.fine] {
                positions.extend(columns.iter().map(|&x| [x, y, z]));
            }
        }
        positions
    }

    /// Every element, positively oriented.
    #[must_use]
    pub fn elements(&self) -> Vec<[u32; 4]> {
        let (columns, layers) = (self.columns().len(), self.layers().len());
        let positions = self.rest_positions();
        let orders = [
            [0, 1, 2],
            [0, 2, 1],
            [1, 0, 2],
            [1, 2, 0],
            [2, 0, 1],
            [2, 1, 0],
        ];
        let mut elements = Vec::with_capacity(6 * (columns - 1) * (layers - 1));
        for k in 0..layers - 1 {
            for i in 0..columns - 1 {
                for order in orders {
                    let mut corner = [i, 0, k];
                    let mut tet = [Self::node(columns, i, 0, k); 4];
                    for (slot, &axis) in order.iter().enumerate() {
                        corner[axis] += 1;
                        tet[slot + 1] = Self::node(columns, corner[0], corner[1], corner[2]);
                    }
                    let x = tet.map(|n| positions[n as usize]);
                    let volume = tet4_volume([
                        x[0][0], x[0][1], x[0][2], x[1][0], x[1][1], x[1][2], x[2][0], x[2][1],
                        x[2][2], x[3][0], x[3][1], x[3][2],
                    ]);
                    if volume < 0.0 {
                        tet.swap(1, 2);
                    }
                    elements.push(tet);
                }
            }
        }
        elements
    }

    /// The lowered model in one `material`: the bottom held, the sides and
    /// top free, and every node held in `y` (plane strain, plan §16d).
    ///
    /// # Errors
    /// [`ModelError::TooLarge`] if the nodes do not fit a `u32` index, and a
    /// [`ModelError`] if the material is out of range.
    pub fn model(&self, material: Material) -> Result<ExplicitModel, ModelError> {
        let positions = self.rest_positions();
        if u32::try_from(positions.len()).is_err() {
            return Err(ModelError::TooLarge);
        }
        let bottom = -self.depth;
        let held = positions.iter().map(|p| p[2] <= bottom).collect();
        let elements = self.elements();
        let count = elements.len();
        let plane = [[0.0, 1.0, 0.0], [0.0; 3]];
        ExplicitModel::new(positions, elements, vec![material; count], held)?
            .with_constraints(vec![plane; self.node_count()])
    }

    /// The number of nodes.
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.columns().len() * 2 * self.layers().len()
    }

    /// The top nodes of row `j` (0 at `y = 0`, 1 at `y = h`), in order of x.
    #[must_use]
    pub fn top_row(&self, j: usize) -> Vec<u32> {
        let columns = self.columns().len();
        (0..columns).map(|i| Self::node(columns, i, j, 0)).collect()
    }

    /// The block's first shear period, `4 depth / c_s`: its bottom is held
    /// and its top is free.
    #[must_use]
    pub fn shear_period(&self, mu: f64, density: f64) -> f64 {
        4.0 * self.depth / (mu / density).sqrt()
    }
}

/// Node coordinates from 0 to `end`: cells of `fine` up to `fine_end`
/// (rounded up to whole cells), then cells growing by a constant ratio,
/// with the count chosen so the last is nearest `coarsest`.
// Cell counts are small; the casts between them and `f64` are exact.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
fn graded(fine: f64, fine_end: f64, end: f64, coarsest: f64) -> Vec<f64> {
    let fine_cells = (fine_end / fine - 1e-9).ceil().max(0.0) as usize;
    let mut nodes: Vec<f64> = (0..=fine_cells).map(|i| i as f64 * fine).collect();
    let rest = end - nodes[fine_cells];
    if rest <= 0.5 * fine {
        return nodes;
    }
    // For `n` cells, the ratio `r` with `fine · (r + r² + … + rⁿ) = rest`.
    let ratio = |n: usize| {
        let span = |r: f64| {
            if r - 1.0 < 1e-12 {
                fine * n as f64
            } else {
                fine * r * (r.powf(n as f64) - 1.0) / (r - 1.0)
            }
        };
        let (mut low, mut high) = (1.0_f64, 2.0_f64);
        for _ in 0..64 {
            if span(high) >= rest {
                break;
            }
            high *= 2.0;
        }
        for _ in 0..200 {
            let middle = 0.5 * (low + high);
            if span(middle) < rest {
                low = middle;
            } else {
                high = middle;
            }
        }
        0.5 * (low + high)
    };
    let most = (rest / fine).floor().max(1.0) as usize;
    let miss = |n: usize| (fine * ratio(n).powf(n as f64) / coarsest).ln().abs();
    let cells = (1..=most)
        .map(|n| (n, miss(n)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map_or(1, |(n, _)| n);
    let r = ratio(cells);
    let mut x = nodes[fine_cells];
    let mut size = fine;
    for _ in 1..cells {
        size *= r;
        x += size;
        nodes.push(x);
    }
    nodes.push(end);
    nodes
}

/// A rigid cylinder, its axis along `y`.
///
/// Its body frame's origin is its lowest point when unturned, and it carries
/// a fixed turn about its own axis (plan §16b: 30°). The turn changes nothing
/// physical, and puts the contact law's frame rotations under test.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cylinder {
    /// The radius `R`.
    pub radius: f64,
    /// The fixed turn about its own axis, in radians.
    pub turn: f64,
}

impl Cylinder {
    /// The pose that puts its lowest point at world `(x, 0, z)`.
    #[must_use]
    pub fn pose(&self, x: f64, z: f64) -> Pose {
        let half = 0.5 * self.turn;
        Pose {
            qw: half.cos(),
            qx: 0.0,
            qy: half.sin(),
            qz: 0.0,
            tx: x,
            ty: 0.0,
            tz: z,
        }
    }

    /// The world's up, `+z`, and its `+x`, in the body frame.
    fn body_axes(&self) -> ([f64; 3], [f64; 3]) {
        let pose = self.pose(0.0, 0.0);
        (
            pose_unrotate(pose, [0.0, 0.0, 1.0]),
            pose_unrotate(pose, [1.0, 0.0, 0.0]),
        )
    }

    /// The exact signed distance to its surface at a body-frame point;
    /// negative inside.
    #[must_use]
    pub fn distance(&self, p: [f64; 3]) -> f64 {
        let (up, _) = self.body_axes();
        (p[0] - self.radius * up[0]).hypot(p[2] - self.radius * up[2]) - self.radius
    }

    /// Its distance baked at `cell` over a body-frame box that holds every
    /// contact on `block`, and past whose faces the clamped lookup reads the
    /// outside for the block's other nodes.
    ///
    /// The box holds the strip within `0.1a` of the plane tangent at the
    /// lowest point, and within `√(0.1 a R)` of that point along it, where the
    /// surface stands `0.05a` clear of the plane. Past a face the lookup reads
    /// the nearest face. So on each face where that moves a point towards the
    /// cylinder (below, and on the side the turn tips the world's up towards)
    /// the box is widened until every point moved there lies `0.1a` below the
    /// plane. The block lies below it, less the press's depth
    /// (`tests/partial_slip.rs` checks every node).
    ///
    /// # Errors
    /// A [`BakeError`] if the grid cannot be baked.
    pub fn baked(&self, block: &Block, cell: f64) -> Result<(SdfGridLayout, Vec<f64>), BakeError> {
        let a = block.contact_half_width;
        let (up, along) = self.body_axes();
        let (reach, band, margin) = ((0.1 * a * self.radius).sqrt(), 0.1 * a, 0.1 * a);
        let (mut low, mut high) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for (s, n) in [
            (-reach, -band),
            (-reach, band),
            (reach, -band),
            (reach, band),
        ] {
            for (axis, index) in [(0, 0), (1, 2)] {
                let x = s * along[index] + n * up[index];
                low[axis] = low[axis].min(x);
                high[axis] = high[axis].max(x);
            }
        }
        let down = |x: f64| (x / cell).floor() * cell;
        let up_to = |x: f64| (x / cell).ceil() * cell;
        let (mut low, mut high) = (low.map(down), high.map(up_to));
        // Clamping up from below `low[1]` raises a point's height above the
        // lowest point (`up · p`), as does clamping x on the side `up[0]`
        // points away from. Each is widened against the other's worst face.
        let (ux, uz) = (up[0], up[2]);
        let worst_x = (ux * low[0]).max(ux * high[0]);
        low[1] = low[1].min(down((-margin - worst_x) / uz));
        let worst_z = uz * high[1];
        if ux > 0.0 {
            low[0] = low[0].min(down((-margin - worst_z) / ux));
        } else if ux < 0.0 {
            high[0] = high[0].max(up_to((-margin - worst_z) / ux));
        }
        bake(
            [low[0], 0.0, low[1]],
            [high[0], up_to(block.fine), high[1]],
            cell,
            |p| self.distance(p),
        )
    }
}

/// Where along the top a zone lies: its two edges' x.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Zone {
    /// The edge at lower x.
    pub left: f64,
    /// The edge at higher x.
    pub right: f64,
}

impl Zone {
    /// Half the zone's width.
    #[must_use]
    pub fn half_width(&self) -> f64 {
        0.5 * (self.right - self.left)
    }
}

/// The contact zone of one row of the top, from each node's mean normal
/// force.
///
/// It is the run of nodes in contact around the largest force, with each
/// edge where the normal force squared, extrapolated linearly from the run's
/// last two nodes, reaches zero (plan §16b). Hertz pressure goes as the
/// square root of the distance to the edge.
///
/// `positions` are the nodes' x, ascending. `None` if no node is in contact.
#[must_use]
pub fn contact_zone(positions: &[f64], normal: &[f64]) -> Option<Zone> {
    zone(positions, normal, 0.0)
}

/// The friction deficit, over its limit `μ_f f_n`, below which a node counts
/// as slipping over a monitor interval.
///
/// A node in the slip zone that sticks on a few of the interval's steps reads
/// a small deficit rather than none (plan §16q). Under the closed forms, a
/// node reads 5 % only within `0.0025 (a² − c²)/c` of the stick zone's edge
/// (arithmetic), and the edge is extrapolated from the nodes inside it.
pub const STICKING_DEFICIT: f64 = 0.05;

/// The stick zone of one row of the top, from each node's mean forces.
///
/// It is the run of sticking nodes around the largest friction deficit
/// `μ_f f_n − f_t`, with each edge where the deficit squared, extrapolated
/// linearly from the run's last two nodes, reaches zero.
///
/// `tangential` is each node's mean friction force along the load's
/// direction. A slipping node's deficit is zero, and a sticking node's
/// approaches zero as the square root of its distance to the edge. A node
/// counts as sticking when its deficit is above [`STICKING_DEFICIT`] of its
/// own limit `μ_f f_n`.
///
/// The deficit, not the ratio `f_t/(μ_f f_n)` the plan first proposed: under
/// the closed forms, the deficit squared is exactly quadratic in x, while the
/// ratio carries the pressure's own square root at the contact's edge
/// (`tests/partial_slip.rs` measures both).
#[must_use]
pub fn stick_zone(
    positions: &[f64],
    normal: &[f64],
    tangential: &[f64],
    friction: f64,
) -> Option<Zone> {
    let deficit: Vec<f64> = normal
        .iter()
        .zip(tangential)
        .map(|(&f_n, &f_t)| {
            let limit = friction * f_n;
            let d = limit - f_t;
            if f_n > 0.0 && d > STICKING_DEFICIT * limit {
                d
            } else {
                0.0
            }
        })
        .collect();
    zone(positions, &deficit, 0.0)
}

/// The run of `values` above `floor` around the largest, with each edge
/// where the value squared, extrapolated linearly from the run's last two
/// nodes, reaches zero. It moves at most one spacing past the run's last
/// node, and stays on it where the square does not rise inwards.
fn zone(positions: &[f64], values: &[f64], floor: f64) -> Option<Zone> {
    let peak = (0..values.len())
        .filter(|&i| values[i] > floor)
        .max_by(|&i, &j| values[i].total_cmp(&values[j]))?;
    let mut first = peak;
    while first > 0 && values[first - 1] > floor {
        first -= 1;
    }
    let mut last = peak;
    while last + 1 < values.len() && values[last + 1] > floor {
        last += 1;
    }
    let edge = |outer: usize, inner: usize, beyond: Option<usize>| {
        let (x1, x2) = (positions[outer], positions[inner]);
        let (g1, g2) = (values[outer].powi(2), values[inner].powi(2));
        let limit = beyond.map_or(x1 + (x1 - x2), |b| positions[b]);
        if outer == inner || g2 <= g1 {
            x1
        } else {
            let x = x1 + g1 * (x1 - x2) / (g2 - g1);
            if x1 < limit {
                x.min(limit)
            } else {
                x.max(limit)
            }
        }
    };
    let before = first.checked_sub(1);
    let after = (last + 1 < values.len()).then_some(last + 1);
    Some(Zone {
        left: edge(first, (first + 1).min(last), before),
        right: edge(last, last.saturating_sub(1).max(first), after),
    })
}

/// A leg of K6's loading (plan §16b).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Leg {
    /// Pressing, until the contact half-width reaches `a`, and the hold after.
    Press,
    /// Pushing along `+x`, until `Q = 0.8 μ_f P`, and the hold after.
    Push,
    /// Moving back along `−x`, until `Q = 0`, and the hold after.
    Return,
}

/// One row of the top, read over one monitor interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RowReading {
    /// The contact zone.
    pub contact: Option<Zone>,
    /// The stick zone along the leg's direction: `+x` while pressing and
    /// pushing, `−x` while returning.
    pub stick: Option<Zone>,
}

impl RowReading {
    /// The stick zone's half-width over the contact's; `None` out of contact.
    #[must_use]
    pub fn stick_over_contact(&self) -> Option<f64> {
        let contact = self.contact?.half_width();
        (contact > 0.0).then(|| self.stick.map_or(0.0, |z| z.half_width()) / contact)
    }
}

/// K6's readout over one monitor interval (plan §16b): each node's forces
/// averaged over the interval, positions at the interval's mean.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// The time at the interval's end.
    pub time: f64,
    /// The leg it belongs to.
    pub leg: Leg,
    /// `P`: the top's normal contact forces, summed.
    pub normal_force: f64,
    /// `Q`: the top's friction forces along `+x`, summed.
    pub tangential_force: f64,
    /// The rows at `y = 0` and `y = h`. The Kuhn split is not symmetric front
    /// to back, so the rows need not agree (plan §16b).
    pub rows: [RowReading; 2],
}

/// K6's criterion, per row: the largest `|c/a − closed form|` over the
/// samples whose load fraction lies in the band (plan §16b: 0.2 to 0.8).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StickErrors {
    /// While the load rises, against [`stick_while_loading`]; `None` if no
    /// sample fell in the band.
    pub loading: [Option<f64>; 2],
    /// While it falls, against [`stick_while_unloading`].
    pub unloading: [Option<f64>; 2],
}

impl StickErrors {
    /// The worse row's largest error, over both phases.
    #[must_use]
    pub fn worst(&self) -> Option<f64> {
        let all = self.loading.iter().chain(&self.unloading);
        all.copied()
            .collect::<Option<Vec<f64>>>()
            .map(|e| e.into_iter().fold(0.0, f64::max))
    }
}

/// What a [`PartialSlipRun`] produced.
#[derive(Clone, Debug)]
pub struct PartialSlipResult {
    /// Every monitor interval's reading, in order.
    pub samples: Vec<Sample>,
    /// `Q` when the return began.
    pub peak_tangential_force: f64,
    /// How far the cylinder's lowest point went below the block's top.
    pub press_depth: f64,
    /// The run's friction coefficient.
    pub friction: f64,
    /// Mean kinetic over mean internal energy over the push and the return.
    pub kinetic_over_internal: Option<f64>,
    /// The energy balance's largest error over the peak internal energy.
    pub energy_balance: Option<f64>,
    /// Whether any element reached `J ≤ 0` (K4).
    pub inverted: bool,
    /// The deepest penetration any node reached (G2).
    pub max_penetration: f64,
    /// Steps taken.
    pub steps: u64,
    /// The last step size.
    pub dt: f64,
    /// Every monitor read.
    pub monitors: Vec<crate::stepping::Sample>,
}

impl PartialSlipResult {
    /// Each sample's load fraction and phase, where K6 judges it: the push's
    /// `Q/(μ_f P)`, and the return's `(Q_peak − Q)/(μ_f P)`.
    #[must_use]
    pub fn fraction(&self, sample: &Sample) -> Option<f64> {
        let limit = self.friction * sample.normal_force;
        match sample.leg {
            Leg::Press => None,
            Leg::Push => Some(sample.tangential_force / limit),
            Leg::Return => Some((self.peak_tangential_force - sample.tangential_force) / limit),
        }
    }

    /// K6's errors over the samples whose load fraction lies in
    /// `[from, to]`.
    #[must_use]
    pub fn errors(&self, from: f64, to: f64) -> StickErrors {
        let mut errors = StickErrors {
            loading: [None; 2],
            unloading: [None; 2],
        };
        for sample in &self.samples {
            let Some(fraction) = self.fraction(sample).filter(|f| (from..=to).contains(f)) else {
                continue;
            };
            let (expected, slot) = match sample.leg {
                Leg::Push => (stick_while_loading(fraction), &mut errors.loading),
                Leg::Return => (stick_while_unloading(fraction), &mut errors.unloading),
                Leg::Press => continue,
            };
            for (row, error) in sample.rows.iter().zip(slot.iter_mut()) {
                let measured = row.stick_over_contact().unwrap_or(0.0);
                let e = (measured - expected).abs();
                *error = Some(error.map_or(e, |m: f64| m.max(e)));
            }
        }
        errors
    }
}

/// Why a [`PartialSlipRun`] did not produce a result.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum PartialSlipError {
    /// The model could not be built.
    #[error(transparent)]
    Model(#[from] ModelError),
    /// The cylinder could not be baked.
    #[error(transparent)]
    Bake(#[from] BakeError),
    /// A pose track was refused.
    #[error(transparent)]
    Obstacle(#[from] ObstacleError),
    /// The run blew up.
    #[error(transparent)]
    Run(#[from] RunError),
    /// A leg did not reach its end within the longest leg allowed.
    #[error("the {leg:?} leg did not reach its end by time {time}")]
    LegDidNotEnd {
        /// The leg.
        leg: Leg,
        /// When it was given up.
        time: f64,
    },
}

/// One run of K6 (plan §16b): press until the contact half-width is `a`,
/// push until `Q = peak · μ_f P`, move back until `Q = 0`, each leg followed
/// by a hold, with every monitor interval read.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PartialSlipRun {
    /// The block.
    pub block: Block,
    /// The cylinder.
    pub cylinder: Cylinder,
    /// The shear modulus μ.
    pub mu: f64,
    /// Poisson's ratio ν.
    pub poisson: f64,
    /// The Kelvin–Voigt viscosity over μ, `η/μ` in seconds; 0 is elastic.
    pub viscous_time: f64,
    /// The density ρ.
    pub density: f64,
    /// The friction coefficient `μ_f`.
    pub friction: f64,
    /// The peak tangential load over `μ_f P` (plan §16b: 0.8).
    pub peak: f64,
    /// The cylinder's speed while pressing.
    pub press_speed: f64,
    /// Its speed while pushing and returning.
    pub push_speed: f64,
    /// The time each leg takes to reach its speed, from rest.
    pub ramp: f64,
    /// The hold after each leg.
    pub hold: f64,
    /// A leg that runs longer than this fails the run.
    pub longest_leg: f64,
    /// The grid's cell (plan §16b: `h/2`).
    pub grid_cell: f64,
}

impl PartialSlipRun {
    /// Plan §16b's run with `a/h = divisions`: `a` 1 mm, the cylinder of
    /// radius 100a turned 30°, the silicone's μ (23 kPa) at ν 0.49 and no
    /// viscosity (the closed forms are elastic), `μ_f` 0.3 and a peak of
    /// 0.8. The loading is the rate ladder's (plan §16q), in units of the
    /// block's first shear period `T`: pressed at 0.004a/T, pushed and
    /// returned at 0.0015a/T, each leg reaching its speed and stopping over
    /// `T`, and held for 2T.
    #[must_use]
    pub fn plan(divisions: f64) -> Self {
        let (a, mu, density) = (1.0e-3, 23.0e3, 1070.0);
        let block = Block::plan(a, divisions);
        let period = block.shear_period(mu, density);
        Self {
            block,
            cylinder: Cylinder {
                radius: 100.0 * a,
                turn: 30_f64.to_radians(),
            },
            mu,
            poisson: 0.49,
            viscous_time: 0.0,
            density,
            friction: 0.3,
            peak: 0.8,
            press_speed: 0.004 * a / period,
            push_speed: 0.0015 * a / period,
            ramp: period,
            hold: 2.0 * period,
            longest_leg: 20.0 * period,
            grid_cell: block.fine / 2.0,
        }
    }

    /// The same run with every speed divided by `factor` and every time
    /// multiplied by it: the rate ladder's rungs.
    #[must_use]
    pub fn slowed(mut self, factor: f64) -> Self {
        self.press_speed /= factor;
        self.push_speed /= factor;
        self.ramp *= factor;
        self.hold *= factor;
        self.longest_leg *= factor;
        self
    }

    /// The run's material: λ from ν, and the viscosity from `η/μ`.
    #[must_use]
    pub fn material(&self) -> Material {
        let nu = self.poisson;
        Material {
            mu: self.mu,
            lambda: self.mu * 2.0 * nu / (1.0 - 2.0 * nu),
            c2: 0.0,
            viscosity: self.viscous_time * self.mu,
            density: self.density,
        }
    }

    /// The cylinder as the run's obstacle, resting on the block's top at the
    /// origin.
    ///
    /// # Errors
    /// A [`BakeError`] if the grid cannot be baked.
    pub fn obstacle(&self) -> Result<Obstacle, BakeError> {
        let (grid, values) = self.cylinder.baked(&self.block, self.grid_cell)?;
        Ok(Obstacle {
            grid,
            values,
            start: 0.0,
            interval: 1.0,
            poses: vec![self.cylinder.pose(0.0, 0.0)],
            friction: self.friction,
        })
    }

    /// Run it on the executor `make` builds.
    ///
    /// Mass damping is `2 ξ ω₀` with `ξ = 0.05` and `ω₀` the block's first
    /// shear frequency, as the tube's is at its own (plan §15c).
    ///
    /// # Errors
    /// A [`PartialSlipError`] if the model or the obstacle cannot be built,
    /// the run blows up, or a leg does not end.
    pub fn run<E: Executor>(
        &self,
        make: impl FnOnce(&ExplicitModel, &Obstacle) -> E,
    ) -> Result<PartialSlipResult, PartialSlipError> {
        let model = self.block.model(self.material())?;
        let obstacle = self.obstacle()?;
        let damping = 2.0 * 0.05 * TAU / self.block.shear_period(self.mu, self.density);
        let mut stepper = Stepper::new(make(&model, &obstacle), StepperConfig::new(damping), 0.0);
        let rows = [self.block.top_row(0), self.block.top_row(1)];
        let mut samples = Vec::new();
        let mut at = [0.0, 0.0];
        let mut peak_tangential_force = 0.0;
        let mut press_depth = 0.0;
        let mut judged_from = f64::INFINITY;
        for leg in [Leg::Press, Leg::Push, Leg::Return] {
            let (heading, speed, direction) = match leg {
                Leg::Press => ([0.0, -1.0], self.press_speed, 1.0),
                Leg::Push => ([1.0, 0.0], self.push_speed, 1.0),
                Leg::Return => ([-1.0, 0.0], self.push_speed, -1.0),
            };
            if leg == Leg::Return {
                peak_tangential_force = samples.last().map_or(0.0, |s: &Sample| s.tangential_force);
            }
            if leg == Leg::Push {
                judged_from = stepper.time();
            }
            let start = stepper.time();
            let track = self.track(start, at, heading, speed, false);
            stepper
                .executor_mut()
                .set_poses(start, track.interval, &track.poses)?;
            let mut previous: Option<(f64, f64)> = None;
            loop {
                let sample = self.interval(&mut stepper, &model, &rows, leg, direction)?;
                samples.push(sample);
                // Stopping takes the ramp, and goes on half as far as the leg
                // would at its speed: stop when that would reach the end.
                if let Some(remaining) = self.remaining(leg, &sample) {
                    let rate = previous.map_or(0.0, |(r, t)| (remaining - r) / (sample.time - t));
                    if remaining + 0.5 * rate.max(0.0) * self.ramp >= 0.0 {
                        break;
                    }
                    previous = Some((remaining, sample.time));
                }
                if stepper.time() - start > self.longest_leg {
                    return Err(PartialSlipError::LegDidNotEnd {
                        leg,
                        time: stepper.time(),
                    });
                }
            }
            // Stop over the ramp, from where it is and as fast as it goes.
            let now = stepper.time();
            let here = track.at(now);
            let moving = speed * ((now - start) / self.ramp).min(1.0);
            let stop = self.track(now, [here.tx, here.tz], heading, moving, true);
            stepper
                .executor_mut()
                .set_poses(now, stop.interval, &stop.poses)?;
            let last = stop.poses[stop.poses.len() - 1];
            at = [last.tx, last.tz];
            if leg == Leg::Press {
                press_depth = -last.tz;
            }
            let until = now + self.ramp + self.hold;
            loop {
                if stepper.time() >= until {
                    break;
                }
                let sample = self.interval(&mut stepper, &model, &rows, leg, direction)?;
                samples.push(sample);
            }
        }
        let monitors = stepper.samples().to_vec();
        Ok(PartialSlipResult {
            samples,
            peak_tangential_force,
            press_depth,
            friction: self.friction,
            kinetic_over_internal: gates::kinetic_over_internal(
                &monitors,
                judged_from,
                stepper.time(),
            ),
            energy_balance: gates::energy_balance(&monitors),
            inverted: gates::inverted(&monitors),
            max_penetration: monitors.last().map_or(0.0, |s| s.monitors.max_penetration),
            steps: stepper.steps(),
            dt: stepper.dt(),
            monitors,
        })
    }

    /// How far `leg` is past its end at `sample`, negative before it, in a
    /// measure that grows about linearly with the leg's travel: while
    /// pressing, the square of the rows' mean contact half-width over `a²`,
    /// less 1 (Hertz: `a² ∝ P`); while pushing, `Q/(μ_f P)` less the peak;
    /// while returning, `−Q/(μ_f P)`. `None` before both rows touch.
    fn remaining(&self, leg: Leg, sample: &Sample) -> Option<f64> {
        let fraction = sample.tangential_force / (self.friction * sample.normal_force);
        match leg {
            Leg::Press => {
                let [front, back] = sample.rows.map(|r| r.contact.map(|z| z.half_width()));
                let a = self.block.contact_half_width;
                front
                    .zip(back)
                    .map(|(f, b)| (0.5 * (f + b) / a).powi(2) - 1.0)
            }
            Leg::Push => Some(fraction - self.peak),
            Leg::Return => Some(-fraction),
        }
    }

    /// The cylinder's track from `start`, its lowest point from `from` (x, z)
    /// along `heading`, sampled every `ramp/10`.
    ///
    /// Starting, it goes from rest to `speed` over `ramp` and holds that speed
    /// for the longest leg. Stopping, it goes from `speed` to rest over
    /// `ramp`, and is then held.
    // The sample count is a few thousand.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )]
    fn track(
        &self,
        start: f64,
        from: [f64; 2],
        heading: [f64; 2],
        speed: f64,
        stopping: bool,
    ) -> Track {
        let interval = self.ramp / 10.0;
        let span = if stopping {
            self.ramp
        } else {
            self.longest_leg
        };
        let count = (span / interval).ceil() as usize + 2;
        let poses = (0..count)
            .map(|s| {
                let t = s as f64 * interval;
                let travel = if stopping {
                    let t = t.min(self.ramp);
                    speed * (t - 0.5 * t * t / self.ramp)
                } else if t < self.ramp {
                    0.5 * speed * t * t / self.ramp
                } else {
                    speed * (t - 0.5 * self.ramp)
                };
                self.cylinder
                    .pose(from[0] + heading[0] * travel, from[1] + heading[1] * travel)
            })
            .collect();
        Track {
            start,
            interval,
            poses,
        }
    }

    /// The steps up to the loop's next monitor read, with the window open,
    /// and their reading.
    fn interval<E: Executor>(
        &self,
        stepper: &mut Stepper<E>,
        model: &ExplicitModel,
        rows: &[Vec<u32>; 2],
        leg: Leg,
        direction: f64,
    ) -> Result<Sample, RunError> {
        stepper.open_window();
        let reads = stepper.samples().len();
        while stepper.samples().len() == reads {
            stepper.step()?;
        }
        stepper.close_window();
        let snapshot = stepper.executor_mut().snapshot();
        let (mut normal_force, mut tangential_force) = (0.0, 0.0);
        let rows = rows.clone().map(|nodes| {
            let row = read_row(model, &snapshot, &nodes);
            normal_force += row.normal.iter().sum::<f64>();
            tangential_force += row.along_x.iter().sum::<f64>();
            let along: Vec<f64> = row.along_x.iter().map(|&f| direction * f).collect();
            RowReading {
                contact: contact_zone(&row.positions, &row.normal),
                stick: stick_zone(&row.positions, &row.normal, &along, self.friction),
            }
        });
        Ok(Sample {
            time: stepper.time(),
            leg,
            normal_force,
            tangential_force,
            rows,
        })
    }
}

/// A leg's pose track.
struct Track {
    start: f64,
    interval: f64,
    poses: Vec<Pose>,
}

impl Track {
    /// The pose at `time`, interpolated as an executor interpolates it.
    // A track is a few thousand samples.
    #[allow(clippy::cast_possible_truncation)]
    fn at(&self, time: f64) -> Pose {
        let span = pose_sample_span(time, self.start, self.interval, self.poses.len() as u32);
        pose_interpolate(
            self.poses[span.lower as usize],
            self.poses[span.upper as usize],
            span.fraction,
        )
    }
}

/// One row's nodes over a window: mean x, mean normal force, and mean
/// friction along `+x`, taken as the friction's size with the sign of its x
/// part, so a slipping node's is exactly `μ_f` times its normal force.
struct Row {
    positions: Vec<f64>,
    normal: Vec<f64>,
    along_x: Vec<f64>,
}

// Step counts stay far below 2^52, where u64 → f64 starts to round.
#[allow(clippy::cast_precision_loss)]
fn read_row(model: &ExplicitModel, snapshot: &Snapshot, nodes: &[u32]) -> Row {
    let steps = snapshot.accumulated_steps as f64;
    let mean = |sum: f64| sum / steps;
    Row {
        positions: nodes
            .iter()
            .map(|&n| {
                let n = n as usize;
                model.rest_positions()[n][0] + mean(snapshot.displacement_sums[n][0])
            })
            .collect(),
        normal: nodes
            .iter()
            .map(|&n| mean(snapshot.normal_force_sums[n as usize]))
            .collect(),
        along_x: nodes
            .iter()
            .map(|&n| {
                let f = snapshot.friction_sums[n as usize].map(mean);
                let size = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
                size.copysign(f[0])
            })
            .collect(),
    }
}
