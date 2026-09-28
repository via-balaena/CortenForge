//! The scan's path: the rigid pose fitted to sliding along the canal (plan §16t, §16w).
//!
//! The **slide** carries every point of the scan along the centreline by the tip's walk, keeping its place in the
//! centreline's frame, which parallel transport carries along the centreline's smoothed tangent. It bends the scan
//! to follow the curve, so it is not a rigid motion. The **fitted pose** at a walk is the rigid motion closest to
//! the slide: least squares over the scan's surface that the slide puts inside the device, each vertex weighted by
//! its area.
//!
//! The walk is the tip's arc from its seat, growing outward: 0 is seated. The centreline runs from the seated tip
//! (its first point) toward the device's mouth, and carries straight on past both ends along its end segments.
//!
//! This was measured with a copy in `tools/cf-sim-research` (`path_room.rs`), which this replaces; its arithmetic is
//! kept, and each vertex's arc and place in the frame are found once rather than for every pose.

use mesh_types::IndexedMesh;
use nalgebra::{
    Isometry3, Matrix3, Point3, Rotation3, Translation3, Unit, UnitQuaternion, Vector3,
};
use sim_soft_explicit::executor::Obstacle;
use sim_soft_explicit::f64::{Pose, pose_interpolate, pose_to_world};

use super::hold::Plane;
use crate::obstacle::{BakedObstacle, SignedDistance, at_each};

/// Why a centreline cannot carry a path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CentrelineError {
    /// Fewer than two points, a point that is not finite, or a first or last segment of no length.
    Degenerate,
}

impl std::fmt::Display for CentrelineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "a centreline needs two or more finite points, and its end segments a length"
        )
    }
}

impl std::error::Error for CentrelineError {}

/// One segment of a centreline with a length.
#[derive(Clone, Copy, Debug)]
struct Segment {
    /// Its first point.
    start: Point3<f64>,
    /// From its first point to its last.
    along: Vector3<f64>,
    /// Its length.
    length: f64,
    /// The arc at its first point: the lengths of every segment before it, summed in order.
    begin: f64,
    /// The arc at its last point: `begin` and its length.
    end: f64,
}

/// A centreline by arc length from the seated tip, carried straight on past both ends, with the frame parallel
/// transport carries along its smoothed tangent.
///
/// A point's tangent weighs its two segments' directions alike, however short either is, so a segment far shorter
/// than its neighbours turns the tangent as far as a long one would: a point repeated a nanometre to one side tilts it
/// 45° (`a_short_segment_turns_the_tangent_as_its_direction_does`). A short segment along the line, as a trim leaves
/// at an end, turns nothing. The centreline is taken as given.
#[derive(Clone, Debug)]
pub struct Centreline {
    points: Vec<Point3<f64>>,
    /// The arc at each point.
    arcs: Vec<f64>,
    /// The frame at each point: two normals, then the smoothed tangent.
    frames: Vec<Matrix3<f64>>,
    /// The segments with a length, in order.
    segments: Vec<Segment>,
    /// The tangent at each segment's ends: `tangents[i]` at segment `i`'s start, `tangents[i + 1]` at its end.
    tangents: Vec<Vector3<f64>>,
}

impl Centreline {
    /// The centreline through `points`, from the seated tip.
    ///
    /// # Errors
    /// [`CentrelineError::Degenerate`] for fewer than two points, a point that is not finite, or a first or last
    /// segment of no length.
    pub fn new(points: &[Point3<f64>]) -> Result<Self, CentrelineError> {
        let long = |a: &Point3<f64>, b: &Point3<f64>| (b - a).norm() >= f64::EPSILON;
        if points.len() < 2
            || !points.iter().all(|p| p.iter().all(|c| c.is_finite()))
            || !long(&points[0], &points[1])
            || !long(&points[points.len() - 2], &points[points.len() - 1])
        {
            return Err(CentrelineError::Degenerate);
        }
        let mut segments = Vec::new();
        let mut walked = 0.0_f64;
        for pair in points.windows(2) {
            let along = pair[1] - pair[0];
            let length = along.norm();
            if length < f64::EPSILON {
                continue;
            }
            segments.push(Segment {
                start: pair[0],
                along,
                length,
                begin: walked,
                end: walked + length,
            });
            walked += length;
        }
        // A point's tangent is the normalised sum of the segments either side of it; an end takes its one
        // segment's, and two exactly opposite neighbours the outgoing one.
        let direction = |i: usize| segments[i].along / segments[i].length;
        let tangents = (0..=segments.len())
            .map(|i| {
                match (
                    i.checked_sub(1).map(direction),
                    segments.get(i).map(|_| direction(i)),
                ) {
                    (Some(a), Some(b)) => {
                        let sum = a + b;
                        if sum.norm() < 1e-12 {
                            b
                        } else {
                            sum.normalize()
                        }
                    }
                    (Some(a), None) => a,
                    (None, Some(b)) => b,
                    (None, None) => Vector3::z(),
                }
            })
            .collect();
        let mut arcs = vec![0.0];
        for pair in points.windows(2) {
            arcs.push(arcs[arcs.len() - 1] + (pair[1] - pair[0]).norm());
        }
        let mut centreline = Self {
            points: points.to_vec(),
            arcs,
            frames: Vec::new(),
            segments,
            tangents,
        };
        let first = centreline.tangent(0.0);
        let seed = if first.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        let normal = (seed - first * seed.dot(&first)).normalize();
        let mut frames = vec![Matrix3::from_columns(&[
            normal,
            first.cross(&normal),
            first,
        ])];
        for &arc in &centreline.arcs[1..] {
            let tangent = centreline.tangent(arc);
            frames.push(turned(&frames[frames.len() - 1], &tangent));
        }
        centreline.frames = frames;
        Ok(centreline)
    }

    /// Its length.
    #[must_use]
    pub fn length(&self) -> f64 {
        self.arcs[self.arcs.len() - 1]
    }

    /// The segment arc `s` falls in: the first whose end is at or past it, or the last.
    fn segment_at(&self, s: f64) -> usize {
        self.segments
            .partition_point(|segment| segment.end < s)
            .min(self.segments.len() - 1)
    }

    /// The point at arc `s`: along the first segment before the tip, along the last past the end.
    #[must_use]
    pub fn point(&self, s: f64) -> Point3<f64> {
        if s < 0.0 {
            let first = self.segments[0];
            return self.points[0] + first.along / first.length * s;
        }
        let i = self.segment_at(s);
        let segment = self.segments[i];
        if s <= segment.end {
            let t = ((s - segment.begin) / segment.length).clamp(0.0, 1.0);
            return Point3::from(segment.start.coords + segment.along * t);
        }
        // Past the end: on along the last segment.
        let last = self.points[self.points.len() - 1];
        last + segment.along / segment.length * (s - self.segments[self.segments.len() - 1].end)
    }

    /// The smoothed tangent at arc `s`: within a segment, its two ends' tangents blended by the fraction of it
    /// covered; before the tip the first segment's, past the end the last's.
    #[must_use]
    pub fn tangent(&self, s: f64) -> Vector3<f64> {
        let s = s.max(0.0);
        let i = self.segment_at(s);
        let segment = self.segments[i];
        if s > segment.end {
            return segment.along / segment.length;
        }
        let f = ((s - segment.begin) / segment.length).clamp(0.0, 1.0);
        let blended = self.tangents[i] * (1.0 - f) + self.tangents[i + 1] * f;
        if blended.norm() < 1e-12 {
            segment.along / segment.length
        } else {
            blended.normalize()
        }
    }

    /// The frame at arc `s`: two normals, then the tangent. Within a segment the smoothed tangent turns on one
    /// great circle, so turning the frame at the segment's start straight onto it is the parallel transport.
    #[must_use]
    pub fn frame(&self, s: f64) -> Matrix3<f64> {
        let below = self.arcs.partition_point(|&arc| arc <= s).saturating_sub(1);
        turned(&self.frames[below], &self.tangent(s))
    }

    /// The arc of the point on the centreline closest to `p`, the centreline carried straight on past both ends.
    #[must_use]
    pub fn arc_of(&self, p: Point3<f64>) -> f64 {
        let last = self.points.len() - 2;
        let mut best = (f64::INFINITY, 0.0);
        for (i, pair) in self.points.windows(2).enumerate() {
            let along = pair[1] - pair[0];
            let length = along.norm();
            if length < f64::EPSILON {
                continue;
            }
            let mut u = (p - pair[0]).dot(&along) / (length * length);
            if i > 0 {
                u = u.max(0.0);
            }
            if i < last {
                u = u.min(1.0);
            }
            let distance = (p - (pair[0] + along * u)).norm();
            if distance < best.0 {
                best = (distance, self.arcs[i] + u * length);
            }
        }
        best.1
    }

    /// Where the slide by `walk` carries `p`: `walk` further from the seated tip, at the same place in the frame.
    #[must_use]
    pub fn slid(&self, p: Point3<f64>, walk: f64) -> Point3<f64> {
        self.placed(p).slid(self, walk)
    }

    /// `p`'s arc and its place in the frame there.
    fn placed(&self, p: Point3<f64>) -> Placed {
        let s = self.arc_of(p);
        Placed {
            arc: s,
            local: self.frame(s).transpose() * (p - self.point(s)),
        }
    }
}

/// A point's arc on a centreline and its place in the frame there: what the slide keeps.
#[derive(Clone, Copy, Debug)]
struct Placed {
    arc: f64,
    local: Vector3<f64>,
}

impl Placed {
    /// Where the slide by `walk` carries it.
    fn slid(self, centreline: &Centreline, walk: f64) -> Point3<f64> {
        let s = self.arc + walk;
        centreline.point(s) + centreline.frame(s) * self.local
    }
}

/// The smallest rotation taking the direction `from` onto `to`; `None` when they are opposite, where no axis is
/// picked out. Its angle is `atan2(|a × b|, a · b)`, which rounding cannot push past its domain as `acos` can.
fn turn_between(from: &Vector3<f64>, to: &Vector3<f64>) -> Option<Rotation3<f64>> {
    let (a, b) = (from.normalize(), to.normalize());
    let axis = a.cross(&b);
    let angle = axis.norm().atan2(a.dot(&b));
    match Unit::try_new(axis, f64::EPSILON) {
        Some(axis) => Some(Rotation3::from_axis_angle(&axis, angle)),
        None if a.dot(&b) < 0.0 => None,
        None => Some(Rotation3::identity()),
    }
}

/// `frame` turned by the smallest rotation that takes its tangent onto `tangent`.
fn turned(frame: &Matrix3<f64>, tangent: &Vector3<f64>) -> Matrix3<f64> {
    let rotation =
        turn_between(&frame.column(2).into_owned(), tangent).unwrap_or_else(Rotation3::identity);
    rotation.matrix() * frame
}

/// The rigid motion that carries each `from` point closest to its `to` point (Kabsch).
///
/// Least squares, weighted by `weights`. `None` when the weights sum to zero, a point is not finite, or the
/// decomposition fails.
#[must_use]
pub fn fitted_motion(
    from: &[Point3<f64>],
    to: &[Point3<f64>],
    weights: &[f64],
) -> Option<Isometry3<f64>> {
    fit(from, to, weights).map(|(motion, _)| motion)
}

/// [`fitted_motion`], and the singular values of the cross-covariance it decomposes.
fn fit(
    from: &[Point3<f64>],
    to: &[Point3<f64>],
    weights: &[f64],
) -> Option<(Isometry3<f64>, Vector3<f64>)> {
    let total: f64 = weights.iter().sum();
    if total <= 0.0 {
        return None;
    }
    let mean = |points: &[Point3<f64>]| {
        points
            .iter()
            .zip(weights)
            .fold(Vector3::zeros(), |sum, (p, w)| sum + p.coords * *w)
            / total
    };
    let (a, b) = (mean(from), mean(to));
    let mut covariance = Matrix3::zeros();
    for ((p, q), w) in from.iter().zip(to).zip(weights) {
        covariance += (p.coords - a) * (q.coords - b).transpose() * *w;
    }
    // `svd` iterates until it converges, and never does on a non-finite matrix.
    if !covariance.iter().all(|x| x.is_finite()) {
        return None;
    }
    let svd = covariance.try_svd(true, true, f64::EPSILON, 1000)?;
    let (u, v) = (svd.u?, svd.v_t?.transpose());
    let mut sign = Matrix3::identity();
    if (v * u.transpose()).determinant() < 0.0 {
        let smallest = svd.singular_values.imin();
        sign[(smallest, smallest)] = -1.0;
    }
    // U and V are orthogonal and the sign makes the product proper, so it is a rotation as it stands.
    let rotation = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
        v * sign * u.transpose(),
    ));
    Some((
        Isometry3::from_parts(Translation3::from(b - rotation * a), rotation),
        svd.singular_values,
    ))
}

/// Each vertex's share of the surface: a third of each triangle it is a corner of.
#[must_use]
pub fn vertex_areas(mesh: &IndexedMesh) -> Vec<f64> {
    let mut areas = vec![0.0; mesh.vertices.len()];
    for face in &mesh.faces {
        let [a, b, c] = face.map(|v| mesh.vertices[v as usize]);
        let third = (b - a).cross(&(c - a)).norm() / 6.0;
        for v in face {
            areas[*v as usize] += third;
        }
    }
    areas
}

/// The spacing of the walks the join and the start are searched on (plan §16w).
pub const WALK_GRID: f64 = 0.000_5;

/// The fit is used while its cross-covariance's second singular value is above this fraction of its first: the
/// points it fits do not lie on one line.
pub const SUPPORT: f64 = 1e-6;

/// The sampling's first count of intervals.
pub const FIRST_INTERVALS: u32 = 64;

/// The most intervals the sampling doubles to before it fails.
pub const MOST_INTERVALS: u32 = 4_096;

/// Why a path cannot be built or sampled.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PathError {
    /// The fit is not used at the seat: fewer than three of the scan's vertices with area lie inside the device, or
    /// they lie on one line, or the fit's decomposition fails.
    NoFitAtTheSeat,
    /// Between the seat and the join, at a walk off the search grid, the fit is not used.
    Unsupported {
        /// The walk.
        walk: f64,
    },
    /// No walk up to this one clears the wall by the clearance.
    NoStart {
        /// The last walk searched.
        searched_to: f64,
    },
    /// A number out of range: a loading time or bar that is not positive and finite, a negative start, or a
    /// clearance that is not finite and non-negative.
    Parameters,
    /// At the most intervals, the interpolated pose is still this far from the fitted one.
    TooCoarse {
        /// The intervals.
        intervals: u32,
        /// The largest distance read.
        error: f64,
    },
}

impl std::fmt::Display for PathError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoFitAtTheSeat => write!(f, "the fit has nothing to fit at the seat"),
            Self::Unsupported { walk } => {
                write!(f, "the fit is not used at walk {walk} m, before the join")
            }
            Self::NoStart { searched_to } => {
                write!(
                    f,
                    "no walk up to {searched_to} m clears the wall by the clearance"
                )
            }
            Self::Parameters => write!(
                f,
                "the loading time and the bar must be positive and finite, the start not negative, and the clearance \
                 finite and not negative"
            ),
            Self::TooCoarse { intervals, error } => write!(
                f,
                "at {intervals} intervals the interpolated pose is still {error} m from the fitted one"
            ),
        }
    }
}

impl std::error::Error for PathError {}

/// The scan's path into the device: the fitted pose, and past the join, where the fit has nothing to fit, the
/// join's rotation moving along the centreline's direction there (plan §16w).
///
/// One per scan: nothing here depends on the wall.
#[derive(Clone, Debug)]
pub struct FittedPath {
    centreline: Centreline,
    /// The scan's vertices with area, where they sit seated.
    vertices: Vec<Point3<f64>>,
    /// Each one's place on the centreline.
    placed: Vec<Placed>,
    /// Each one's area.
    areas: Vec<f64>,
    /// The device's cap planes, their normals out of it.
    device: Vec<Plane>,
    /// The join and the pose there, if the fit ever stops being used.
    join: Option<(f64, Isometry3<f64>)>,
}

impl FittedPath {
    /// The path of `scan`, seated, along `centreline` into the device bounded by `device`'s planes, each normal
    /// pointing out of it. It finds the join on the [`WALK_GRID`], out to the centreline's length and the diagonal of
    /// the scan's box.
    ///
    /// # Errors
    /// [`PathError::NoFitAtTheSeat`] if the fit is not used at the seat.
    pub fn new(
        scan: &IndexedMesh,
        centreline: Centreline,
        device: Vec<Plane>,
    ) -> Result<Self, PathError> {
        let areas = vertex_areas(scan);
        let (vertices, areas): (Vec<Point3<f64>>, Vec<f64>) = scan
            .vertices
            .iter()
            .zip(areas)
            .filter(|&(_, area)| area > 0.0)
            .map(|(p, area)| (*p, area))
            .unzip();
        let placed = at_each(vertices.len(), |i| centreline.placed(vertices[i]));
        let mut path = Self {
            centreline,
            vertices,
            placed,
            areas,
            device,
            join: None,
        };
        if path.fitted(0.0).is_none() {
            return Err(PathError::NoFitAtTheSeat);
        }
        let reach = path.centreline.length() + path.diameter();
        // Whole grid steps out to the reach.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let steps = (reach / WALK_GRID).ceil() as usize;
        let grid = |k: usize| {
            #[allow(clippy::cast_precision_loss)] // far fewer than 2^52 steps
            let k = k as f64;
            k * WALK_GRID
        };
        let supported = at_each(steps + 1, |k| path.fitted(grid(k)).is_some());
        if let Some(first) = supported.iter().position(|&used| !used) {
            let walk = grid(first - 1);
            let pose = path.fitted(walk).ok_or(PathError::Unsupported { walk })?;
            path.join = Some((walk, pose));
        }
        Ok(path)
    }

    /// The centreline.
    #[must_use]
    pub const fn centreline(&self) -> &Centreline {
        &self.centreline
    }

    /// The join, if the fit ever stops being used.
    #[must_use]
    pub fn join(&self) -> Option<f64> {
        self.join.map(|(walk, _)| walk)
    }

    /// Whether `q` is inside the device: on the inner side of every cap plane.
    #[must_use]
    pub fn inside(&self, q: Point3<f64>) -> bool {
        self.device.iter().all(|plane| plane.height(q) < 0.0)
    }

    /// The diagonal of the box around the scan's vertices with area: at least the largest distance between two of
    /// them.
    fn diameter(&self) -> f64 {
        let Some(first) = self.vertices.first() else {
            return 0.0;
        };
        let (low, high) = self
            .vertices
            .iter()
            .fold((*first, *first), |(low, high), p| (low.inf(p), high.sup(p)));
        (high - low).norm()
    }

    /// The fitted pose at `walk`: `None` where the fit is not used (fewer than three of the scan's vertices with area
    /// slid inside the device, or those on one line).
    #[must_use]
    pub fn fitted(&self, walk: f64) -> Option<Isometry3<f64>> {
        let (mut from, mut to, mut weights) = (Vec::new(), Vec::new(), Vec::new());
        for ((p, placed), area) in self.vertices.iter().zip(&self.placed).zip(&self.areas) {
            let q = placed.slid(&self.centreline, walk);
            if self.inside(q) {
                from.push(*p);
                to.push(q);
                weights.push(*area);
            }
        }
        if from.len() < 3 {
            return None;
        }
        let (motion, spread) = fit(&from, &to, &weights)?;
        let mut spread = [spread.x, spread.y, spread.z];
        spread.sort_by(|a, b| b.total_cmp(a));
        (spread[1] > SUPPORT * spread[0]).then_some(motion)
    }

    /// The path's pose at `walk`: the fitted pose up to the join, and past it the join's rotation, moved along the
    /// centreline's direction at the join.
    ///
    /// # Errors
    /// [`PathError::Unsupported`] where the fit is not used before the join, between the grid's walks.
    pub fn pose(&self, walk: f64) -> Result<Isometry3<f64>, PathError> {
        match self.join {
            Some((join, pose)) if walk > join => {
                let along = self.centreline.tangent(join) * (walk - join);
                Ok(Isometry3::from_parts(
                    Translation3::from(pose.translation.vector + along),
                    pose.rotation,
                ))
            }
            _ => self.fitted(walk).ok_or(PathError::Unsupported { walk }),
        }
    }

    /// Where the run starts: the first walk on the [`WALK_GRID`], from the join outward (from the seat, with no
    /// join), at which every one of `wall` lies at least `clearance` outside the scan by `scan`'s signed distance.
    ///
    /// # Errors
    /// [`PathError::Parameters`] for a clearance that is not finite and non-negative; [`PathError::NoStart`] if no
    /// walk clears the wall out to the join, the diagonal of the scan's box and the clearance; and
    /// [`PathError::Unsupported`] as [`Self::pose`].
    pub fn start(
        &self,
        wall: &[Point3<f64>],
        scan: &SignedDistance,
        clearance: f64,
    ) -> Result<f64, PathError> {
        if !(clearance.is_finite() && clearance >= 0.0) {
            return Err(PathError::Parameters);
        }
        let from = self.join().unwrap_or(0.0);
        let last = from + self.diameter() + clearance;
        // Whole grid steps out to the last walk; far fewer than 2^52, so each is exact.
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        for step in 0..=((last - from) / WALK_GRID).floor() as u64 {
            let candidate = (step as f64).mul_add(WALK_GRID, from);
            let pose = self.pose(candidate)?;
            let closest = at_each(wall.len(), |n| {
                scan.signed(pose.inverse_transform_point(&wall[n]))
            })
            .into_iter()
            .fold(f64::INFINITY, f64::min);
            if closest >= clearance {
                return Ok(candidate);
            }
        }
        Err(PathError::NoStart { searched_to: last })
    }

    /// The path from `start` to the seat over a loading of `loading_time`, sampled evenly in time: the count of
    /// intervals doubles from [`FIRST_INTERVALS`] until the interpolated pose lies within `bar` of the path at every
    /// scan vertex the path puts inside the device, read at a quarter, a half and three quarters of every interval.
    ///
    /// # Errors
    /// [`PathError::Parameters`] for a loading time or bar that is not positive and finite or a negative start;
    /// [`PathError::TooCoarse`] if [`MOST_INTERVALS`] do not meet the bar; and [`PathError::Unsupported`] as
    /// [`Self::pose`].
    pub fn sampled(
        &self,
        start: f64,
        loading_time: f64,
        bar: f64,
    ) -> Result<SampledPath, PathError> {
        let positive = |x: f64| x.is_finite() && x > 0.0;
        if !(positive(loading_time) && positive(bar) && start.is_finite() && start >= 0.0) {
            return Err(PathError::Parameters);
        }
        // The path at a quarter of the finest interval: `u` in `[0, 1]` on a grid of `4 · MOST_INTERVALS`.
        let finest = 4 * MOST_INTERVALS;
        let mut known: Vec<Option<Isometry3<f64>>> = vec![None; finest as usize + 1];
        let walk_at = |index: u32| start - travelled(f64::from(index) / f64::from(finest), start);
        let mut errors = Vec::new();
        let mut intervals = FIRST_INTERVALS;
        loop {
            // The samples and the reading points at this count, found where not yet known.
            let step = finest / (4 * intervals);
            let needed: Vec<u32> = (0..=4 * intervals)
                .map(|k| k * step)
                .filter(|&i| known[i as usize].is_none())
                .collect();
            let found = at_each(needed.len(), |n| self.pose(walk_at(needed[n])));
            for (i, pose) in needed.into_iter().zip(found) {
                known[i as usize] = Some(pose?);
            }
            let at = |k: u32| known[(k * step) as usize].map(to_pose);
            let poses: Vec<Pose> = (0..=intervals).filter_map(|k| at(4 * k)).collect();
            let error = at_each(intervals as usize, |k| {
                let k = u32::try_from(k).unwrap_or(u32::MAX);
                let (a, b) = (poses[k as usize], poses[k as usize + 1]);
                [1_u32, 2, 3]
                    .into_iter()
                    .filter_map(|quarter| {
                        let path = at(4 * k + quarter)?;
                        let between = pose_interpolate(a, b, f64::from(quarter) / 4.0);
                        Some(self.strayed(path, between))
                    })
                    .fold(0.0, f64::max)
            })
            .into_iter()
            .fold(0.0, f64::max);
            errors.push((intervals, error));
            if error <= bar {
                return Ok(SampledPath {
                    interval: loading_time / f64::from(intervals),
                    poses,
                    errors,
                });
            }
            if intervals >= MOST_INTERVALS {
                return Err(PathError::TooCoarse { intervals, error });
            }
            intervals *= 2;
        }
    }

    /// The farthest `between` puts any scan vertex with area from where `path` puts it, over the vertices `path`
    /// puts inside the device.
    fn strayed(&self, path: Pose, between: Pose) -> f64 {
        self.vertices
            .iter()
            .filter_map(|p| {
                let on_path = pose_to_world(path, [p.x, p.y, p.z]);
                self.inside(Point3::from(on_path)).then(|| {
                    let off = pose_to_world(between, [p.x, p.y, p.z]);
                    Vector3::from(off).metric_distance(&Vector3::from(on_path))
                })
            })
            .fold(0.0, f64::max)
    }
}

/// A path sampled evenly in time from time 0, as an obstacle takes it.
#[derive(Clone, Debug, PartialEq)]
pub struct SampledPath {
    /// The time between samples.
    pub interval: f64,
    /// The poses, body to world, from the start to the seat.
    pub poses: Vec<Pose>,
    /// At each count of intervals tried, the farthest the interpolated pose strayed from the path.
    pub errors: Vec<(u32, f64)>,
}

impl SampledPath {
    /// The obstacle `baked` posed along this path from time 0, at friction `friction`; held at the seat after it.
    #[must_use]
    pub fn obstacle(&self, baked: BakedObstacle, friction: f64) -> Obstacle {
        Obstacle {
            grid: baked.grid,
            values: baked.values,
            fine: Some(baked.fine),
            start: 0.0,
            interval: self.interval,
            poses: self.poses.clone(),
            friction,
        }
    }
}

/// How far the tip has gone, of `travel`, a fraction `u` of the way through the loading: plan §15b's profile, whose
/// speed ramps up over the first tenth and down over the last.
#[must_use]
pub fn travelled(u: f64, travel: f64) -> f64 {
    let ramp = 0.1;
    let top_speed = travel / 0.9;
    let acceleration = top_speed / ramp;
    let u = u.clamp(0.0, 1.0);
    if u <= ramp {
        0.5 * acceleration * u * u
    } else if u <= 1.0 - ramp {
        (0.5 * top_speed).mul_add(ramp, top_speed * (u - ramp))
    } else {
        let remaining = 1.0 - u;
        (0.5 * acceleration * remaining).mul_add(-remaining, travel)
    }
}

/// An isometry as the solver's pose.
fn to_pose(isometry: Isometry3<f64>) -> Pose {
    let (q, t) = (isometry.rotation, isometry.translation.vector);
    Pose {
        qw: q.w,
        qx: q.i,
        qy: q.j,
        qz: q.k,
        tx: t.x,
        ty: t.y,
        tz: t.z,
    }
}
