//! Plan §15b's tube on a mandrel: the mesh, the mandrel's distance field, the
//! insertion, and the band readout K2 judges.
//!
//! Lengths are in metres, pressures in pascals. The tube's axis is `z`; its
//! entry is at `z = 0` and its far end at `z = L`. The mandrel enters from
//! `z < 0`, moving along `+z`.

use std::f64::consts::TAU;

use crate::executor::{Obstacle, Snapshot};
use crate::f64::{Material, Pose, SdfGridLayout, tet4_volume, triangle_area};
use crate::{ExplicitModel, ModelError};

/// One of plan §15c's pre-registered meshes (`n_r × n_θ × n_z` cells).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mesh {
    /// 3 × 32 × 17 cells, 9 792 elements.
    TenK,
    /// 5 × 48 × 35 cells, 50 400 elements.
    FiftyK,
    /// 6 × 64 × 43 cells, 99 072 elements.
    HundredK,
}

/// How the tube is held (plan §15b, 15d.8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Walls {
    /// The far end held, the entry and the outer wall free: the free-ends
    /// oracle applies (plan §15b).
    Free,
    /// The outer wall held by a rigid case, and all axial motion held: the
    /// oracle's cased wall in plane strain (plan 15d.8).
    ///
    /// The outer wall is held whole, not only radially. A radial hold is a
    /// fixed direction per node, so a node sliding around the tube moves
    /// along its tangent line, and so outward. Nothing else stops the tube
    /// rotating, and held that way the confined wall rotated and opened its
    /// case instead of compressing (plan §16n). The axisymmetric problem has
    /// no motion around the tube, so holding the wall whole poses the same
    /// problem.
    Cased,
}

/// A structured annular mesh, with nodes exactly on circles.
///
/// Each cell is split into six tetrahedra around the same body diagonal (the
/// Kuhn split of `hand_built.rs`), so shared faces match, across the wrap in
/// θ too.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tube {
    /// The inner radius `A`.
    pub inner_radius: f64,
    /// The outer radius `B`.
    pub outer_radius: f64,
    /// The length `L`.
    pub length: f64,
    /// Cells through the wall.
    pub radial: usize,
    /// Cells around.
    pub circumferential: usize,
    /// Cells along.
    pub axial: usize,
}

impl Tube {
    /// Plan §15b's tube (A 10 mm, B 20 mm, L 120 mm) on `mesh`.
    #[must_use]
    pub const fn plan(mesh: Mesh) -> Self {
        let (radial, circumferential, axial) = match mesh {
            Mesh::TenK => (3, 32, 17),
            Mesh::FiftyK => (5, 48, 35),
            Mesh::HundredK => (6, 64, 43),
        };
        Self {
            inner_radius: 0.010,
            outer_radius: 0.020,
            length: 0.120,
            radial,
            circumferential,
            axial,
        }
    }

    /// The number of nodes.
    #[must_use]
    pub const fn node_count(&self) -> usize {
        (self.radial + 1) * self.circumferential * (self.axial + 1)
    }

    /// The number of elements.
    #[must_use]
    pub const fn element_count(&self) -> usize {
        6 * self.radial * self.circumferential * self.axial
    }

    /// The node at radial level `i`, angle index `j` (taken around the
    /// circle) and axial level `k`. Meaningful only for a tube whose nodes
    /// fit a `u32` index, which [`Tube::model`] checks.
    // Truncation needs over 4 billion nodes; `model` refuses such a tube.
    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub const fn node(&self, i: usize, j: usize, k: usize) -> u32 {
        let j = j % self.circumferential;
        ((k * self.circumferential + j) * (self.radial + 1) + i) as u32
    }

    /// A node's `(i, j, k)` levels.
    #[must_use]
    pub const fn levels(&self, node: usize) -> (usize, usize, usize) {
        let n = node;
        let i = n % (self.radial + 1);
        let rest = n / (self.radial + 1);
        (i, rest % self.circumferential, rest / self.circumferential)
    }

    /// The radius, angle and height of levels `(i, j, k)`.
    // Level counts are small; `usize → f64` is exact for them.
    #[allow(clippy::cast_precision_loss)]
    #[must_use]
    pub fn cylindrical(&self, i: usize, j: usize, k: usize) -> (f64, f64, f64) {
        let wall = self.outer_radius - self.inner_radius;
        (
            self.inner_radius + wall * i as f64 / self.radial as f64,
            TAU * j as f64 / self.circumferential as f64,
            self.length * k as f64 / self.axial as f64,
        )
    }

    /// Every node's rest position.
    #[must_use]
    pub fn rest_positions(&self) -> Vec<[f64; 3]> {
        let mut positions = vec![[0.0; 3]; self.node_count()];
        for k in 0..=self.axial {
            for j in 0..self.circumferential {
                for i in 0..=self.radial {
                    let (r, theta, z) = self.cylindrical(i, j, k);
                    positions[self.node(i, j, k) as usize] = [r * theta.cos(), r * theta.sin(), z];
                }
            }
        }
        positions
    }

    /// Every element, positively oriented.
    #[must_use]
    pub fn elements(&self) -> Vec<[u32; 4]> {
        let positions = self.rest_positions();
        let orders = [
            [0, 1, 2],
            [0, 2, 1],
            [1, 0, 2],
            [1, 2, 0],
            [2, 0, 1],
            [2, 1, 0],
        ];
        let mut elements = Vec::with_capacity(self.element_count());
        for k in 0..self.axial {
            for j in 0..self.circumferential {
                for i in 0..self.radial {
                    for order in orders {
                        let mut corner = [i, j, k];
                        let mut tet = [self.node(i, j, k); 4];
                        for (slot, &axis) in order.iter().enumerate() {
                            corner[axis] += 1;
                            tet[slot + 1] = self.node(corner[0], corner[1], corner[2]);
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
        }
        elements
    }

    /// The lowered model in one `material`, held as `walls` says.
    ///
    /// # Errors
    /// [`ModelError::TooLarge`] if the nodes do not fit a `u32` index, and a
    /// [`ModelError`] if the material is out of range.
    pub fn model(&self, material: Material, walls: Walls) -> Result<ExplicitModel, ModelError> {
        if u32::try_from(self.node_count()).is_err() {
            return Err(ModelError::TooLarge);
        }
        let positions = self.rest_positions();
        let elements = self.elements();
        let count = elements.len();
        let held: Vec<bool> = (0..self.node_count())
            .map(|n| {
                let (i, _, k) = self.levels(n);
                match walls {
                    Walls::Free => k == self.axial,
                    Walls::Cased => i == self.radial,
                }
            })
            .collect();
        let model = ExplicitModel::new(positions, elements, vec![material; count], held)?;
        match walls {
            Walls::Free => Ok(model),
            Walls::Cased => {
                let axial = [[0.0, 0.0, 1.0], [0.0; 3]];
                model.with_constraints(vec![axial; self.node_count()])
            }
        }
    }

    /// The inner-surface nodes with rest height in `[from, to]`: plan
    /// §15b's band is `[20, 60]` mm.
    #[must_use]
    pub fn band(&self, from: f64, to: f64) -> Vec<u32> {
        let mut nodes = Vec::new();
        for k in 0..=self.axial {
            let (_, _, z) = self.cylindrical(0, 0, k);
            if (from..=to).contains(&z) {
                nodes.extend((0..self.circumferential).map(|j| self.node(0, j, k)));
            }
        }
        nodes
    }
}

/// A rigid mandrel: a cylinder with a hemispherical nose of its own radius.
/// In its body frame the tip is at the origin and the axis runs along `−z`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mandrel {
    /// The radius `a`.
    pub radius: f64,
}

impl Mandrel {
    /// The exact signed distance to the surface at a body-frame point;
    /// negative inside.
    #[must_use]
    pub fn distance(&self, p: [f64; 3]) -> f64 {
        let nose_centre = -self.radius;
        if p[2] <= nose_centre {
            p[0].hypot(p[1]) - self.radius
        } else {
            p[0].hypot(p[1]).hypot(p[2] - nose_centre) - self.radius
        }
    }

    /// The distance baked into a grid of spacing `cell` over the body-frame
    /// box `[low, high]` (plan §15c pins `cell` at A/20).
    ///
    /// # Errors
    /// A [`BakeError`] unless `cell` is positive, the box is ordered on every
    /// axis, each side is a whole number of cells, and the grid holds at most
    /// `u32::MAX` samples.
    pub fn baked(
        &self,
        low: [f64; 3],
        high: [f64; 3],
        cell: f64,
    ) -> Result<(SdfGridLayout, Vec<f64>), BakeError> {
        if !(cell.is_finite() && cell > 0.0) {
            return Err(BakeError::Cell);
        }
        let mut sizes = [0_u32; 3];
        for axis in 0..3 {
            let cells = (high[axis] - low[axis]) / cell;
            if !(cells.is_finite() && cells >= 0.0) {
                return Err(BakeError::Box);
            }
            if (cells - cells.round()).abs() > 1e-9 * cells.max(1.0) {
                return Err(BakeError::NotWholeCells { axis });
            }
            // A whole, non-negative number of cells; the product check below
            // bounds it.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let samples = cells.round().min(f64::from(u32::MAX - 1)) as u32 + 1;
            sizes[axis] = samples;
        }
        let total = sizes.iter().map(|&n| u64::from(n)).product::<u64>();
        if total > u64::from(u32::MAX) {
            return Err(BakeError::TooLarge);
        }
        let grid = SdfGridLayout {
            origin_x: low[0],
            origin_y: low[1],
            origin_z: low[2],
            cell_size: cell,
            size_x: sizes[0],
            size_y: sizes[1],
            size_z: sizes[2],
        };
        let mut values = Vec::with_capacity((grid.size_x * grid.size_y * grid.size_z) as usize);
        for k in 0..grid.size_z {
            for j in 0..grid.size_y {
                for i in 0..grid.size_x {
                    values.push(self.distance([
                        low[0] + f64::from(i) * cell,
                        low[1] + f64::from(j) * cell,
                        low[2] + f64::from(k) * cell,
                    ]));
                }
            }
        }
        Ok((grid, values))
    }
}

/// Why a mandrel could not be baked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum BakeError {
    /// The cell size is not positive and finite.
    #[error("the cell size must be positive and finite")]
    Cell,
    /// The box is not ordered (`low ≤ high`) or not finite.
    #[error("the box must be finite with low <= high on every axis")]
    Box,
    /// A side is not a whole number of cells, so the far face would miss
    /// `high`.
    #[error("the box's side along axis {axis} is not a whole number of cells")]
    NotWholeCells {
        /// The axis, 0 to 2.
        axis: usize,
    },
    /// The grid would hold more than `u32::MAX` samples.
    #[error("the grid would hold more than u32::MAX samples")]
    TooLarge,
}

/// Plan §15b's insertion of the mandrel's tip.
///
/// The tip starts `start_gap` before the entry and travels
/// `start_gap + depth`: its speed ramps up over the first 10 % of the loading
/// time, holds for 80 %, and ramps down over the last 10 %; then it is held.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Insertion {
    /// How far before the entry the tip starts (plan: 5 mm).
    pub start_gap: f64,
    /// How far past the entry it stops (plan: 100 mm).
    pub depth: f64,
    /// The loading time `T`.
    pub loading_time: f64,
    /// The hold after loading (plan: 0.2 s).
    pub hold: f64,
}

impl Insertion {
    /// The plan's insertion over loading time `loading_time`.
    #[must_use]
    pub const fn plan(loading_time: f64) -> Self {
        Self {
            start_gap: 0.005,
            depth: 0.100,
            loading_time,
            hold: 0.2,
        }
    }

    /// The end of the run: loading and hold.
    #[must_use]
    pub fn end(&self) -> f64 {
        self.loading_time + self.hold
    }

    /// The tip's height at time `t`.
    #[must_use]
    pub fn tip(&self, t: f64) -> f64 {
        let travel = self.start_gap + self.depth;
        let t_total = self.loading_time;
        let ramp = 0.1 * t_total;
        let top_speed = travel / (0.9 * t_total);
        let acceleration = top_speed / ramp;
        let t = t.clamp(0.0, t_total);
        let covered = if t <= ramp {
            0.5 * acceleration * t * t
        } else if t <= t_total - ramp {
            0.5 * top_speed * ramp + top_speed * (t - ramp)
        } else {
            let remaining = t_total - t;
            travel - 0.5 * acceleration * remaining * remaining
        };
        covered - self.start_gap
    }

    /// The mandrel as an obstacle: its distance baked at `cell` over the
    /// region the tube can reach, and its pose sampled every `T/1000`.
    ///
    /// The box is rounded out to whole cells. Beyond it the clamped lookup
    /// reads the nearest face, which is outside the mandrel wherever a tube
    /// node can be.
    ///
    /// # Errors
    /// A [`BakeError`] if the grid cannot be baked.
    pub fn obstacle(
        &self,
        mandrel: Mandrel,
        tube: &Tube,
        cell: f64,
        friction: f64,
    ) -> Result<Obstacle, BakeError> {
        let whole = |length: f64| (length / cell).ceil() * cell;
        let reach = whole(1.25 * tube.outer_radius);
        let (behind, ahead) = (whole(self.depth + 0.005), whole(0.005));
        let (grid, values) =
            mandrel.baked([-reach, -reach, -behind], [reach, reach, ahead], cell)?;
        let interval = self.loading_time / 1000.0;
        let samples = 1001;
        let poses = (0..samples)
            .map(|s| Pose {
                qw: 1.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                tx: 0.0,
                ty: 0.0,
                tz: self.tip(f64::from(s) * interval),
            })
            .collect();
        Ok(Obstacle {
            grid,
            values,
            start: 0.0,
            interval,
            poses,
            friction,
        })
    }
}

/// What the band reads, from one window's sums (plan 15d.1).
#[derive(Clone, Debug, PartialEq)]
pub struct BandReading {
    /// The band's area-weighted mean contact pressure, `ΣF / ΣA`.
    pub pressure: f64,
    /// Ring pressures (`ΣF / ΣA` per axial level), averaged over each pair of
    /// adjacent levels, since single levels alternate (plan §15c).
    pub paired_levels: Vec<f64>,
    /// The standard deviation of the band's node pressures over their mean.
    pub node_scatter: f64,
    /// The band's mean gap to the true mandrel surface, deformed radius
    /// minus `a`: negative when the nodes sit inside it.
    pub gap: f64,
    /// The band's axial stretch `λ_z`, from its first and last levels.
    pub axial_stretch: f64,
}

/// Read the band `nodes` (from [`Tube::band`]) from a window's `snapshot`.
///
/// Everything is a window mean, so the pressure and the reference it is
/// judged against come from the same interval: each node's pressure is its
/// mean normal force over its tributary area (a third of each incident
/// boundary triangle, plan §15c), with the areas, the gap and `λ_z` taken at
/// the mean displaced position.
///
/// # Panics
/// If the snapshot holds no accumulated steps, or the band is empty.
// Counts of nodes and levels are small; `usize → f64` is exact for them.
#[allow(clippy::cast_precision_loss)]
#[must_use]
pub fn read_band(
    tube: &Tube,
    model: &ExplicitModel,
    snapshot: &Snapshot,
    mandrel: Mandrel,
    nodes: &[u32],
) -> BandReading {
    assert!(snapshot.accumulated_steps > 0, "the window is empty");
    assert!(!nodes.is_empty(), "the band is empty");
    let steps = snapshot.accumulated_steps as f64;
    let deformed = |n: usize| {
        let (x, sum) = (model.rest_positions()[n], snapshot.displacement_sums[n]);
        [
            x[0] + sum[0] / steps,
            x[1] + sum[1] / steps,
            x[2] + sum[2] / steps,
        ]
    };
    let area = |n: usize| tributary_area(model, snapshot, n);
    let readings: Vec<(u32, f64, f64)> = nodes
        .iter()
        .map(|&n| {
            (
                n,
                snapshot.normal_force_sums[n as usize] / steps,
                area(n as usize),
            )
        })
        .collect();
    let (force, total_area) = readings
        .iter()
        .fold((0.0, 0.0), |(f, a), &(_, nf, na)| (f + nf, a + na));
    let pressure = force / total_area;

    let mut levels: Vec<usize> = readings
        .iter()
        .map(|&(n, _, _)| tube.levels(n as usize).2)
        .collect();
    levels.sort_unstable();
    levels.dedup();
    let level_pressure = |k: usize| {
        let (f, a) = readings
            .iter()
            .filter(|&&(n, _, _)| tube.levels(n as usize).2 == k)
            .fold((0.0, 0.0), |(f, a), &(_, nf, na)| (f + nf, a + na));
        f / a
    };
    let rings: Vec<f64> = levels.iter().map(|&k| level_pressure(k)).collect();
    let paired_levels = rings.windows(2).map(|w| 0.5 * (w[0] + w[1])).collect();

    let node_pressures: Vec<f64> = readings.iter().map(|&(_, f, a)| f / a).collect();
    let mean = node_pressures.iter().sum::<f64>() / node_pressures.len() as f64;
    let variance = node_pressures
        .iter()
        .map(|p| (p - mean).powi(2))
        .sum::<f64>()
        / node_pressures.len() as f64;

    let gap = nodes
        .iter()
        .map(|&n| {
            let p = deformed(n as usize);
            p[0].hypot(p[1]) - mandrel.radius
        })
        .sum::<f64>()
        / nodes.len() as f64;

    let (first, last) = (levels[0], levels[levels.len() - 1]);
    let mean_height = |k: usize| {
        let heights: Vec<f64> = (0..=tube.radial)
            .flat_map(|i| (0..tube.circumferential).map(move |j| (i, j)))
            .map(|(i, j)| deformed(tube.node(i, j, k) as usize)[2])
            .collect();
        heights.iter().sum::<f64>() / heights.len() as f64
    };
    let rest_span = tube.cylindrical(0, 0, last).2 - tube.cylindrical(0, 0, first).2;
    let axial_stretch = (mean_height(last) - mean_height(first)) / rest_span;

    BandReading {
        pressure,
        paired_levels,
        node_scatter: variance.sqrt() / mean,
        gap,
        axial_stretch,
    }
}

/// A surface node's tributary area at the window's mean state: a third of
/// each incident boundary triangle (plan §15c).
///
/// # Panics
/// If the snapshot holds no accumulated steps.
// Step counts stay far below 2^52, where u64 → f64 starts to round.
#[allow(clippy::cast_precision_loss)]
#[must_use]
pub fn tributary_area(model: &ExplicitModel, snapshot: &Snapshot, node: usize) -> f64 {
    assert!(snapshot.accumulated_steps > 0, "the window is empty");
    let steps = snapshot.accumulated_steps as f64;
    let mean = |n: usize| {
        let (x, sum) = (model.rest_positions()[n], snapshot.displacement_sums[n]);
        [
            x[0] + sum[0] / steps,
            x[1] + sum[1] / steps,
            x[2] + sum[2] / steps,
        ]
    };
    model
        .surface_incidence()
        .of(node)
        .iter()
        .map(|&slot| {
            let [a, b, c] = model.surface_triangles()[slot as usize / 3];
            triangle_area(mean(a as usize), mean(b as usize), mean(c as usize)) / 3.0
        })
        .sum()
}

/// Each of `nodes`' window-mean contact pressure: its mean normal force over
/// its [`tributary_area`].
///
/// # Panics
/// If the snapshot holds no accumulated steps.
// Step counts stay far below 2^52, where u64 → f64 starts to round.
#[allow(clippy::cast_precision_loss)]
#[must_use]
pub fn node_pressures(model: &ExplicitModel, snapshot: &Snapshot, nodes: &[u32]) -> Vec<f64> {
    assert!(snapshot.accumulated_steps > 0, "the window is empty");
    let steps = snapshot.accumulated_steps as f64;
    nodes
        .iter()
        .map(|&n| {
            snapshot.normal_force_sums[n as usize]
                / steps
                / tributary_area(model, snapshot, n as usize)
        })
        .collect()
}

/// An oracle case: plan §15b's free-ends tube, or 15d.8's cased tube in
/// plane strain, with the oracle's answer and its sensitivities
/// (`docs/soft_contact/thick_tube_reference.py`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TubeCase {
    /// `a / A`.
    pub mandrel_ratio: f64,
    /// `B / A`.
    pub thickness_ratio: f64,
    /// Poisson's ratio ν.
    pub poisson: f64,
    /// Yeoh's C₂ over μ; 0 is neo-Hookean (the Yeoh case, plan 16h).
    pub c2_over_mu: f64,
    /// How the tube is held.
    pub walls: Walls,
    /// The contact pressure over μ.
    pub pressure_over_mu: f64,
    /// The axial stretch `λ_z` (1 for the cased tube).
    pub axial_stretch: f64,
    /// `∂(p/μ)/∂(a/A)` at fixed `λ_z`.
    pub pressure_per_mandrel_ratio: f64,
    /// `∂(p/μ)/∂λ_z` at fixed `a`.
    pub pressure_per_axial_stretch: f64,
}

/// K2's two errors for a reading against its case (plan 15d.1).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Errors {
    /// Against the oracle at radius `a`, relative.
    pub raw: f64,
    /// Against the oracle at `a` plus the band's mean gap, relative; `None`
    /// for the cased tube, where the linearization in the gap is off by
    /// 3.5 % at §15c's predicted 0.36 mm (a review's measurement against
    /// the exact oracle), and §15g records only the raw error.
    pub gap_corrected: Option<f64>,
}

impl TubeCase {
    /// The errors of `reading` for a tube of inner radius `inner_radius` in a
    /// material of shear modulus `mu`. Both references are the oracle's
    /// linearization at the band's measured `λ_z`.
    #[must_use]
    pub fn errors(&self, reading: &BandReading, inner_radius: f64, mu: f64) -> Errors {
        let at_stretch = self.pressure_over_mu
            + self.pressure_per_axial_stretch * (reading.axial_stretch - self.axial_stretch);
        let at_gap = at_stretch + self.pressure_per_mandrel_ratio * reading.gap / inner_radius;
        let measured = reading.pressure / mu;
        Errors {
            raw: measured / at_stretch - 1.0,
            gap_corrected: (self.walls == Walls::Free).then(|| measured / at_gap - 1.0),
        }
    }
}

/// One run of the tube on the mandrel: plan §15b's case, loading and window.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TubeRun {
    /// Which mesh.
    pub mesh: Mesh,
    /// The oracle case the run is judged against.
    pub case: TubeCase,
    /// The shear modulus μ; λ follows from the case's ν, and Yeoh's C₂ from
    /// its C₂/μ.
    pub mu: f64,
    /// The density ρ.
    pub density: f64,
    /// The mandrel's motion (plan §15b: [`Insertion::plan`]).
    pub insertion: Insertion,
    /// The measurement window's length, at the end of the hold (plan §15b:
    /// 0.1 s).
    pub window: f64,
    /// The friction coefficient `μ_f`.
    pub friction: f64,
    /// The mandrel grid's cell size (plan §15c pins A/20).
    pub grid_cell: f64,
}

/// What a [`TubeRun`] produced.
#[derive(Clone, Debug)]
pub struct TubeResult {
    /// The band's reading over the measurement window.
    pub reading: BandReading,
    /// K2's errors against the case.
    pub errors: Errors,
    /// Mean kinetic over mean internal energy in the window.
    pub kinetic_over_internal: Option<f64>,
    /// The energy balance's largest error over the peak internal energy.
    pub energy_balance: Option<f64>,
    /// Whether any element reached `J ≤ 0` (K4).
    pub inverted: bool,
    /// The deepest penetration any node reached (G2).
    pub max_penetration: f64,
    /// The band's `λ_z` against the oracle's, relative (the validity gate
    /// is 0.5 %).
    pub axial_stretch_error: f64,
    /// Steps taken.
    pub steps: u64,
    /// How many times the stable step was estimated.
    pub estimates: u64,
    /// The last step size.
    pub dt: f64,
    /// Every monitor read; the last follows the last step.
    pub samples: Vec<crate::stepping::Sample>,
    /// The state at the end, with the window's sums, from which
    /// [`node_pressures`] gives any node's window-mean pressure (K5's seated
    /// percentile).
    pub snapshot: Snapshot,
}

/// Why a [`TubeRun`] did not produce a result.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum TubeRunError {
    /// The model could not be built.
    #[error(transparent)]
    Model(#[from] ModelError),
    /// The mandrel could not be baked.
    #[error(transparent)]
    Bake(#[from] BakeError),
    /// The run blew up.
    #[error(transparent)]
    Run(#[from] crate::stepping::RunError),
}

impl TubeRun {
    /// The shear wave's axial period `T_s = 4 L / c_s` (plan §15b), for a
    /// material of shear modulus `mu` and density `density`.
    #[must_use]
    pub fn shear_period(mu: f64, density: f64) -> f64 {
        4.0 * Tube::plan(Mesh::TenK).length / (mu / density).sqrt()
    }

    /// The run's material: λ from the case's ν, and C₂ from its C₂/μ.
    #[must_use]
    pub fn material(&self) -> Material {
        let nu = self.case.poisson;
        Material {
            mu: self.mu,
            lambda: self.mu * 2.0 * nu / (1.0 - 2.0 * nu),
            c2: self.case.c2_over_mu * self.mu,
            density: self.density,
        }
    }

    /// The mandrel for this run's case, on `tube`.
    #[must_use]
    pub fn mandrel(&self, tube: &Tube) -> Mandrel {
        Mandrel {
            radius: self.case.mandrel_ratio * tube.inner_radius,
        }
    }

    /// The mandrel as the run's obstacle: baked at the run's cell size, with
    /// its friction.
    ///
    /// # Errors
    /// A [`BakeError`] if the grid cannot be baked.
    pub fn obstacle(&self, tube: &Tube) -> Result<Obstacle, BakeError> {
        self.insertion
            .obstacle(self.mandrel(tube), tube, self.grid_cell, self.friction)
    }

    /// Run it on the executor `make` builds, and read the band over the
    /// window at the end of the hold (plan §15b).
    ///
    /// Mass damping is `2 ξ ω₀` with `ξ = 0.05` and `ω₀ = 2π / T_s`
    /// (plan §15c), on through the hold (plan §16e).
    ///
    /// # Errors
    /// A [`TubeRunError`] if the model cannot be built, the mandrel cannot
    /// be baked, or the run blows up.
    pub fn run<E: crate::executor::Executor>(
        &self,
        make: impl FnOnce(&ExplicitModel, &Obstacle) -> E,
    ) -> Result<TubeResult, TubeRunError> {
        use crate::stepping::{Stepper, StepperConfig, gates};

        let tube = Tube::plan(self.mesh);
        let model = tube.model(self.material(), self.case.walls)?;
        let mandrel = self.mandrel(&tube);
        let insertion = self.insertion;
        let obstacle = self.obstacle(&tube)?;
        let damping = 2.0 * 0.05 * (TAU / Self::shear_period(self.mu, self.density));
        let mut stepper = Stepper::new(make(&model, &obstacle), StepperConfig::new(damping), 0.0);
        let window = insertion.end() - self.window;
        stepper.run_until(window)?;
        stepper.open_window();
        stepper.run_until(insertion.end())?;
        stepper.close_window();

        let snapshot = stepper.executor_mut().snapshot();
        let band = tube.band(0.020, 0.060);
        let reading = read_band(&tube, &model, &snapshot, mandrel, &band);
        let errors = self.case.errors(&reading, tube.inner_radius, self.mu);
        let samples = stepper.samples().to_vec();
        Ok(TubeResult {
            axial_stretch_error: reading.axial_stretch / self.case.axial_stretch - 1.0,
            kinetic_over_internal: gates::kinetic_over_internal(&samples, window, insertion.end()),
            energy_balance: gates::energy_balance(&samples),
            inverted: gates::inverted(&samples),
            max_penetration: samples.last().map_or(0.0, |s| s.monitors.max_penetration),
            steps: stepper.steps(),
            estimates: stepper.estimates(),
            dt: stepper.dt(),
            reading,
            errors,
            samples,
            snapshot,
        })
    }
}
