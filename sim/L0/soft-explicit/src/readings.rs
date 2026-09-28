//! Fit plan D1's readings: what a verdict reads from a run (plan §16s).
//!
//! **Getting it in** is the peak push force over the path: the largest of the
//! stepping loop's monitor means of the push, the contact force's component
//! along the path (the tube probe reads the tube's axis). Its geometric share,
//! the push of the same run at `μ_f = 0` (plan §15h), rises and falls as the
//! obstacle crosses each ring of nodes; [`travel_peak`] reads it as the
//! largest mean push over [`PUSH_TRAVEL`] of travel, which averages most of
//! the crossings out.
//!
//! **Seated** is [`WindowContact::patch_peak`]: the contact force on the
//! most-loaded patch of [`PROBE_AREA`], 1 cm², divided by that area. 1 cm² is
//! the algometer tip most pressure-pain studies in a 2021 review used (plan
//! §16s), so the reading and the limit it will be judged against can be taken
//! over the same area. The pointwise pressures behind it, their peak and their
//! area-weighted 95th percentile are shown beside it
//! ([`WindowContact::pressures`], [`area_percentile`]).
//!
//! Both are read from window means (plan §15c): a node's mean position, and
//! its mean normal force over the window.
//!
//! **On a path that turns the obstacle** the push is `−Σ fᵢ · (dxᵢ/ds)`, which
//! takes the twist as well as the force (plan §15g's list for steps 6–9). The
//! executor accumulates the work the obstacle's motion does against the
//! contact ([`Monitors::obstacle_work`](crate::executor::Monitors)), and
//! [`work_peak`] reads its largest mean over a window of travel. Beside it:
//! the direction along the path ([`along_path`]), the force across it
//! ([`sideways`]), the moment about a point of the obstacle
//! ([`moment_about`]), and the push at a pose held still ([`static_push`]),
//! plan §16x.

use std::collections::HashMap;
use std::f64::consts::PI;

use crate::ExplicitModel;
use crate::executor::{Obstacle, Snapshot, rigid_motion};
use crate::f64::{
    Pose, pose_rotate, pose_to_world, triangle_area, vec3_add, vec3_cross, vec3_dot, vec3_length,
    vec3_scale, vec3_sub,
};

/// The seated reading's patch: 1 cm², the algometer tip most pressure-pain
/// studies in a 2021 review used (fit plan D1, 2026-09-26; plan §16s). The
/// size follows whichever data calibrates the seated limit.
pub const PROBE_AREA: f64 = 1.0e-4;

/// The travel the geometric share's push is averaged over: 10 mm (plan §16s).
///
/// That is 2.9 and 3.6 ring spacings on plan §15c's 50k and 100k tubes. Over
/// `w` spacings a sinusoidal ripple keeps `|sin πw| / πw` of its amplitude:
/// 3 % and 9 % there (`tests/readings.rs`).
pub const PUSH_TRAVEL: f64 = 0.010;

/// Sub-triangles per patch radius along a triangle's longest edge, when a
/// patch integrates the surface's force (see [`WindowContact::patch_peak`]).
const SUBDIVISION: f64 = 16.0;

/// The largest mean push over any `window` of travel: the work done over the
/// window divided by its length.
///
/// Each sample is the travel at the end of a read interval and the mean force
/// over that interval, taken as constant along it; the first interval starts
/// at `start`. Intervals without travel, such as a hold, add no work.
///
/// `None` if `window` is not positive and finite, or the samples travel less
/// than it or backwards.
#[must_use]
pub fn travel_peak(start: f64, samples: &[(f64, f64)], window: f64) -> Option<f64> {
    if !(window > 0.0 && window.is_finite()) {
        return None;
    }
    let mut travel = Vec::with_capacity(samples.len() + 1);
    let mut work = Vec::with_capacity(samples.len() + 1);
    travel.push(start);
    work.push(0.0);
    for &(end, force) in samples {
        let from = travel[travel.len() - 1];
        if end < from {
            return None;
        }
        work.push(work[work.len() - 1] + force * (end - from));
        travel.push(end);
    }
    let last = travel[travel.len() - 1];
    if last - start < window {
        return None;
    }
    // The work is piecewise linear in the travel, so the windowed mean is
    // too, with corners where either end of the window meets a sample.
    let work_at = |x: f64| {
        let after = travel
            .partition_point(|&t| t < x)
            .clamp(1, travel.len() - 1);
        let (x0, x1) = (travel[after - 1], travel[after]);
        let (w0, w1) = (work[after - 1], work[after]);
        if x1 > x0 {
            w0 + (w1 - w0) * (x - x0) / (x1 - x0)
        } else {
            w1
        }
    };
    travel
        .iter()
        .flat_map(|&t| [t, t + window])
        .filter(|&x| x >= start + window && x <= last)
        .map(|x| (work_at(x) - work_at(x - window)) / window)
        .reduce(f64::max)
}

/// The largest mean push over any `window` of travel, from the cumulative
/// work the obstacle's motion has done against the contact
/// ([`Monitors::obstacle_work`](crate::executor::Monitors)).
///
/// `samples` are `(travel, work)` at each read, the first where the travel
/// starts; between reads the work is taken as linear in the travel, as
/// [`travel_peak`] takes it. A read without travel, as in a hold, adds no
/// work. `None` where [`travel_peak`] is.
#[must_use]
pub fn work_peak(samples: &[(f64, f64)], window: f64) -> Option<f64> {
    let &(start, _) = samples.first()?;
    let means: Vec<(f64, f64)> = samples
        .windows(2)
        .map(|pair| {
            let ((from, before), (to, after)) = (pair[0], pair[1]);
            let travel = to - from;
            (
                to,
                if travel > 0.0 {
                    (after - before) / travel
                } else {
                    0.0
                },
            )
        })
        .collect();
    travel_peak(start, &means, window)
}

/// The unit direction a point of the obstacle at `point`, in its body frame,
/// moves in from pose `from` to pose `to`; `None` if it does not move.
#[must_use]
pub fn along_path(from: Pose, to: Pose, point: [f64; 3]) -> Option<[f64; 3]> {
    let moved = vec3_sub(pose_to_world(to, point), pose_to_world(from, point));
    let length = vec3_length(moved);
    (length > 0.0).then(|| vec3_scale(moved, 1.0 / length))
}

/// `force` less its part along the unit direction `along`.
#[must_use]
pub const fn sideways(force: [f64; 3], along: [f64; 3]) -> [f64; 3] {
    vec3_sub(force, vec3_scale(along, vec3_dot(force, along)))
}

/// A moment about the body origin of `pose`, of forces whose resultant is
/// `force`, taken instead about the obstacle's point at `point` in its body
/// frame: `M − (R · point) × F`.
#[must_use]
pub const fn moment_about(
    moment: [f64; 3],
    force: [f64; 3],
    pose: Pose,
    point: [f64; 3],
) -> [f64; 3] {
    vec3_sub(moment, vec3_cross(pose_rotate(pose, point), force))
}

/// The push at a pose `at` held still, per unit of `advance` over the path's
/// move from `from` to `to`.
///
/// It is the work the contact forces' resultant `force` and their moment
/// `moment`, about the body origin of `at`, would resist over that move:
/// `(F · Δp + φ · (M + (p_at − p_from) × F)) / advance`, with `Δp` and `φ`
/// the move and the turn ([`rigid_motion`]); the moment is carried to
/// `from`'s origin, where the turn is taken. Linear in the turn. `None`
/// unless `advance` is positive.
#[must_use]
pub fn static_push(
    force: [f64; 3],
    moment: [f64; 3],
    at: Pose,
    (from, to): (Pose, Pose),
    advance: f64,
) -> Option<f64> {
    (advance > 0.0).then(|| {
        let (moved, turn) = rigid_motion(from, to);
        let offset = vec3_sub([at.tx, at.ty, at.tz], [from.tx, from.ty, from.tz]);
        let carried = vec3_add(moment, vec3_cross(offset, force));
        (vec3_dot(force, moved) + vec3_dot(turn, carried)) / advance
    })
}

/// The pressure that the most-pressed `fraction` of the area is at or above.
///
/// The `(pressure, area)` pairs are sorted by pressure, highest first, and
/// read where their running area first reaches `fraction` of the total.
/// `None` if there is no area.
#[must_use]
pub fn area_percentile(readings: &[(f64, f64)], fraction: f64) -> Option<f64> {
    let mut sorted = readings.to_vec();
    sorted.sort_by(|a, b| b.0.total_cmp(&a.0));
    let total: f64 = sorted.iter().map(|&(_, area)| area).sum();
    let mut covered = 0.0;
    for &(pressure, area) in &sorted {
        covered += area;
        if total > 0.0 && covered >= fraction * total {
            return Some(pressure);
        }
    }
    None
}

/// The seated reading: the most-loaded patch and where it is centred.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Patch {
    /// The contact force on the patch over its area.
    pub pressure: f64,
    /// The patch's centre, a node or a triangle's centroid.
    pub centre: [f64; 3],
}

/// The contact over a measurement window, from which D1's seated readings
/// are taken. It keeps the model's surface, so its readings need nothing
/// else.
#[derive(Clone, Debug, PartialEq)]
pub struct WindowContact {
    surface: Vec<[u32; 3]>,
    positions: Vec<[f64; 3]>,
    forces: Vec<f64>,
    areas: Vec<f64>,
    /// Each surface triangle's weight at each of its nodes: the share of the
    /// triangle's third that node's contact area takes.
    facing: Vec<[f64; 3]>,
}

impl WindowContact {
    /// Read `snapshot`'s window, with the obstacle's normal taken as it is
    /// posed at `time`, a time in a window over which it is held.
    ///
    /// A node is in contact if it carried normal force in the window. Its
    /// contact area counts each incident boundary triangle by how squarely
    /// the triangle faces the obstacle: a third of it times the cosine between
    /// its outward normal and the obstacle's normal turned inward, and nothing
    /// if it is turned away. So a face at right angles to the obstacle adds
    /// nothing to the node, and a face tilted toward it adds its share by the
    /// cosine. A contact node with no facing area at all, as where the
    /// obstacle meets an edge exactly side-on, takes a third of every incident
    /// triangle instead, so a node with any surface keeps its force.
    ///
    /// Near side-on a node's pressure grows as one over the cosine, without
    /// bound, and rounding decides whether a side-on node has any facing area:
    /// the pointwise pressures are unbounded at a side-on contact. The patch
    /// keeps the force at any angle.
    ///
    /// # Panics
    /// If the snapshot holds no accumulated steps.
    // Step counts stay far below 2^52, where u64 → f64 starts to round.
    #[allow(clippy::cast_precision_loss)]
    #[must_use]
    pub fn read(
        model: &ExplicitModel,
        snapshot: &Snapshot,
        obstacle: &Obstacle,
        time: f64,
    ) -> Self {
        assert!(snapshot.accumulated_steps > 0, "the window is empty");
        let steps = snapshot.accumulated_steps as f64;
        let positions: Vec<[f64; 3]> = model
            .rest_positions()
            .iter()
            .zip(&snapshot.displacement_sums)
            .map(|(x, sum)| {
                [
                    x[0] + sum[0] / steps,
                    x[1] + sum[1] / steps,
                    x[2] + sum[2] / steps,
                ]
            })
            .collect();
        let forces: Vec<f64> = snapshot
            .normal_force_sums
            .iter()
            .map(|sum| sum / steps)
            .collect();
        let normals: Vec<Option<[f64; 3]>> = positions
            .iter()
            .zip(&forces)
            .map(|(&p, &f)| (f > 0.0).then(|| obstacle.world_normal(time, p)))
            .collect();
        let surface = model.surface_triangles().to_vec();
        let thirds: Vec<f64> = surface
            .iter()
            .map(|&corners| {
                let [a, b, c] = corners.map(|n| positions[n as usize]);
                triangle_area(a, b, c) / 3.0
            })
            .collect();
        let mut areas = vec![0.0; positions.len()];
        let mut tributary = vec![0.0; positions.len()];
        let mut facing: Vec<[f64; 3]> = surface
            .iter()
            .zip(&thirds)
            .map(|(&corners, &third)| {
                let [a, b, c] = corners.map(|n| positions[n as usize]);
                let outward = unit(vec3_cross(vec3_sub(b, a), vec3_sub(c, a)));
                corners.map(|n| {
                    tributary[n as usize] += third;
                    normals[n as usize].map_or(0.0, |obstacle_normal| {
                        let cosine = (-vec3_dot(outward, obstacle_normal)).max(0.0);
                        areas[n as usize] += third * cosine;
                        cosine
                    })
                })
            })
            .collect();
        let sideways: Vec<bool> = (0..positions.len())
            .map(|n| forces[n] > 0.0 && areas[n] <= 0.0)
            .collect();
        if sideways.contains(&true) {
            for (corners, weights) in surface.iter().zip(&mut facing) {
                for (&n, weight) in corners.iter().zip(weights) {
                    if sideways[n as usize] {
                        *weight = 1.0;
                    }
                }
            }
            for (n, area) in areas.iter_mut().enumerate() {
                if sideways[n] {
                    *area = tributary[n];
                }
            }
        }
        Self {
            surface,
            positions,
            forces,
            areas,
            facing,
        }
    }

    /// Each node's mean position over the window.
    #[must_use]
    pub fn positions(&self) -> &[[f64; 3]] {
        &self.positions
    }

    /// Each node's mean normal force over the window; zero off the contact.
    #[must_use]
    pub fn forces(&self) -> &[f64] {
        &self.forces
    }

    /// Each node's contact area (see [`read`](Self::read)); zero off the
    /// contact.
    #[must_use]
    pub fn areas(&self) -> &[f64] {
        &self.areas
    }

    /// Each contact node's `(pressure, area)`: its mean normal force over its
    /// contact area.
    #[must_use]
    pub fn pressures(&self) -> Vec<(f64, f64)> {
        self.forces
            .iter()
            .zip(&self.areas)
            .filter(|&(&force, &area)| force > 0.0 && area > 0.0)
            .map(|(&force, &area)| (force / area, area))
            .collect()
    }

    /// The contact force on the most-loaded patch of `area`, over `area`
    /// (fit plan D1's seated reading at [`PROBE_AREA`]); `None` without
    /// contact.
    ///
    /// Each node's force is spread over its contact area, so a surface
    /// triangle carries a uniform pressure: the mean of its nodes' pressures,
    /// each weighted by its share of the triangle. The patch is the surface
    /// inside a ball of radius `√(area/π)`; the triangles are integrated over
    /// it in congruent sub-triangles, at least `SUBDIVISION` per radius along
    /// each longest edge, each counted by its centroid, faded in across the
    /// ball's rim over its own size.
    ///
    /// The patch is centred on every contact node and on the centroid of every
    /// triangle that carries force, so this is the most-loaded of those: a
    /// lower bound on the most-loaded patch anywhere. On a curved surface the
    /// ball holds a little more than `area` of it; `tests/readings.rs`
    /// measures how much on a bore.
    #[must_use]
    pub fn patch_peak(&self, area: f64) -> Option<Patch> {
        let points = self.force_points(area);
        let radius = (area / PI).sqrt();
        let grid = Buckets::new(&points, radius);
        let centroids = self
            .surface
            .iter()
            .filter(|corners| corners.iter().any(|&n| self.forces[n as usize] > 0.0))
            .map(|&corners| centroid(corners.map(|n| self.positions[n as usize])));
        self.positions
            .iter()
            .zip(&self.forces)
            .filter(|&(_, &force)| force > 0.0)
            .map(|(&p, _)| p)
            .chain(centroids)
            .map(|centre| Patch {
                pressure: grid.within(&points, centre, radius, |p| p.force) / area,
                centre,
            })
            .reduce(|best, patch| {
                if patch.pressure > best.pressure {
                    patch
                } else {
                    best
                }
            })
    }

    /// The contact force on a patch of `area` centred at `centre`, over
    /// `area`: the reading [`patch_peak`](Self::patch_peak) maximizes.
    #[must_use]
    pub fn patch_at(&self, area: f64, centre: [f64; 3]) -> f64 {
        let points = self.force_points(area);
        let radius = (area / PI).sqrt();
        Buckets::new(&points, radius).within(&points, centre, radius, |p| p.force) / area
    }

    /// The surface carrying force inside a patch of `area` centred at
    /// `centre`, integrated as [`patch_peak`](Self::patch_peak) integrates
    /// the force: on a curved surface a ball holds more than `area` of it,
    /// and it holds any other loaded surface within its radius too (plan
    /// §16s, §16x).
    #[must_use]
    pub fn loaded_area_at(&self, area: f64, centre: [f64; 3]) -> f64 {
        let points = self.force_points(area);
        let radius = (area / PI).sqrt();
        Buckets::new(&points, radius).within(&points, centre, radius, |p| p.area)
    }

    /// The surface's force as points: each force-carrying triangle cut into
    /// congruent sub-triangles, each a point at its centroid with its share
    /// of the triangle's force and area, and its size, the triangle's longest
    /// edge over the cuts.
    // Sub-triangle counts are small whole numbers.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_precision_loss
    )]
    fn force_points(&self, area: f64) -> Vec<ForcePoint> {
        let spacing = (area / PI).sqrt() / SUBDIVISION;
        let mut points = Vec::new();
        for (&corners, weights) in self.surface.iter().zip(&self.facing) {
            let pressure: f64 = corners
                .iter()
                .zip(weights)
                .map(|(&n, &weight)| {
                    let (force, node_area) = (self.forces[n as usize], self.areas[n as usize]);
                    if force > 0.0 && node_area > 0.0 {
                        force / node_area * weight / 3.0
                    } else {
                        0.0
                    }
                })
                .sum();
            if pressure <= 0.0 {
                continue;
            }
            let triangle = corners.map(|n| self.positions[n as usize]);
            let [first, second, third] = triangle;
            let longest = [
                vec3_sub(second, first),
                vec3_sub(third, second),
                vec3_sub(first, third),
            ]
            .map(vec3_length)
            .into_iter()
            .fold(0.0, f64::max);
            let cuts = (longest / spacing).ceil().max(1.0) as usize;
            let area = triangle_area(first, second, third) / (cuts * cuts) as f64;
            let force = pressure * area;
            let size = longest / cuts as f64;
            points.extend(sub_centroids(triangle, cuts).map(|at| ForcePoint {
                at,
                force,
                area,
                size,
            }));
        }
        points
    }
}

/// A unit vector along `v`, or zero for a degenerate one.
fn unit(v: [f64; 3]) -> [f64; 3] {
    let length = vec3_length(v);
    if length > 0.0 {
        vec3_scale(v, 1.0 / length)
    } else {
        [0.0; 3]
    }
}

/// The centroids of the `cuts²` congruent sub-triangles made by cutting each
/// of `triangle`'s edges into `cuts`: in each row, the upright ones and the
/// inverted ones between them.
// Cut counts are small whole numbers.
#[allow(clippy::cast_precision_loss)]
fn sub_centroids(triangle: [[f64; 3]; 3], cuts: usize) -> impl Iterator<Item = [f64; 3]> {
    let [origin, along, across] = triangle;
    let point = move |steps_along: f64, steps_across: f64| {
        let (toward_along, toward_across) = (steps_along / cuts as f64, steps_across / cuts as f64);
        let toward_origin = 1.0 - toward_along - toward_across;
        [0, 1, 2].map(|axis| {
            toward_origin * origin[axis] + toward_along * along[axis] + toward_across * across[axis]
        })
    };
    (0..cuts).flat_map(move |row| {
        (0..cuts - row).flat_map(move |column| {
            let (row_steps, column_steps) = (row as f64, column as f64);
            let upright = point(row_steps + 1.0 / 3.0, column_steps + 1.0 / 3.0);
            let inverted = (row + column + 1 < cuts)
                .then(|| point(row_steps + 2.0 / 3.0, column_steps + 2.0 / 3.0));
            std::iter::once(upright).chain(inverted)
        })
    })
}

fn centroid([a, b, c]: [[f64; 3]; 3]) -> [f64; 3] {
    [
        (a[0] + b[0] + c[0]) / 3.0,
        (a[1] + b[1] + c[1]) / 3.0,
        (a[2] + b[2] + c[2]) / 3.0,
    ]
}

/// A share of a triangle's force and area at a sub-triangle's centroid.
#[derive(Clone, Copy)]
struct ForcePoint {
    at: [f64; 3],
    force: f64,
    area: f64,
    /// The sub-triangle's size, over which the patch's rim fades it in.
    size: f64,
}

/// Points bucketed in cubes of a patch's radius, so a patch reads only the
/// 27 cubes around its centre.
struct Buckets {
    size: f64,
    cubes: HashMap<[i64; 3], Vec<usize>>,
}

impl Buckets {
    /// Cubes as wide as `radius` plus the largest point, so every point that
    /// counts toward a patch of `radius` is in a cube next to its centre's.
    fn new(points: &[ForcePoint], radius: f64) -> Self {
        let size = radius + points.iter().map(|p| p.size).fold(0.0, f64::max);
        let mut cubes: HashMap<[i64; 3], Vec<usize>> = HashMap::new();
        for (index, point) in points.iter().enumerate() {
            cubes.entry(cube(point.at, size)).or_default().push(index);
        }
        Self { size, cubes }
    }

    /// The `weight` of the points within `radius` of `centre`, their force or
    /// their area. A point within half its size of the rim counts in
    /// proportion to how far inside it is: counted whole or not at all, the
    /// rim's points add noise that the peak over many centres turns into a
    /// bias.
    fn within(
        &self,
        points: &[ForcePoint],
        centre: [f64; 3],
        radius: f64,
        weight: impl Fn(&ForcePoint) -> f64,
    ) -> f64 {
        let [x, y, z] = cube(centre, self.size);
        let mut sum = 0.0;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(members) = self.cubes.get(&[x + dx, y + dy, z + dz]) else {
                        continue;
                    };
                    for &index in members {
                        let point = points[index];
                        let d = vec3_length(vec3_sub(point.at, centre));
                        let inside = ((radius - d) / point.size + 0.5).clamp(0.0, 1.0);
                        sum += inside * weight(&point);
                    }
                }
            }
        }
        sum
    }
}

// Positions are metres at most a few hundred cubes from the origin.
#[allow(clippy::cast_possible_truncation)]
fn cube(p: [f64; 3], size: f64) -> [i64; 3] {
    p.map(|x| (x / size).floor() as i64)
}
