//! Broad-phase collision detection using Sweep-and-Prune (SAP).
//!
//! This module provides efficient O(n log n) broad-phase collision detection
//! to reduce the number of narrow-phase collision tests from O(n²) to O(n + k)
//! where k is the number of overlapping AABB pairs.
//!
//! # Algorithm
//!
//! Sweep-and-Prune (also known as Sort-and-Sweep) works by:
//! 1. Computing axis-aligned bounding boxes (AABBs) for all bodies
//! 2. Projecting AABBs onto each axis (typically just one axis for 1D SAP)
//! 3. Sorting intervals by their minimum endpoint
//! 4. Sweeping through sorted intervals to find overlaps
//!
//! For 3D scenes, we use a single-axis sweep (on the axis with most variance)
//! which is simple and effective for most physics scenarios.
//!
//! # Example
//!
//! ```
//! use sim_core::broad_phase::{BroadPhase, SweepAndPrune};
//! use sim_core::{Body, CollisionShape};
//! use sim_types::{BodyId, RigidBodyState, Pose, MassProperties};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create some bodies
//! let bodies = vec![
//!     Body::new(
//!         BodyId::new(1),
//!         RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 0.0))),
//!         MassProperties::sphere(1.0, 0.5),
//!     ).with_collision_shape(CollisionShape::sphere(1.0)),
//!     Body::new(
//!         BodyId::new(2),
//!         RigidBodyState::at_rest(Pose::from_position(Point3::new(1.5, 0.0, 0.0))),
//!         MassProperties::sphere(1.0, 0.5),
//!     ).with_collision_shape(CollisionShape::sphere(1.0)),
//! ];
//!
//! let mut sap = SweepAndPrune::new();
//! let pairs = sap.find_potential_pairs(&bodies);
//!
//! // Bodies overlap, so we should get a potential pair
//! assert_eq!(pairs.len(), 1);
//! ```

use nalgebra::{Point3, Vector3};
use sim_types::BodyId;

use crate::world::{Body, CollisionShape};

/// An axis-aligned bounding box (AABB) for broad-phase collision detection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    /// Minimum corner of the bounding box.
    pub min: Point3<f64>,
    /// Maximum corner of the bounding box.
    pub max: Point3<f64>,
}

impl Aabb {
    /// Create a new AABB from minimum and maximum corners.
    #[must_use]
    pub const fn new(min: Point3<f64>, max: Point3<f64>) -> Self {
        Self { min, max }
    }

    /// Create an AABB centered at a point with the given half-extents.
    #[must_use]
    pub fn from_center(center: Point3<f64>, half_extents: Vector3<f64>) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    /// Check if this AABB overlaps with another AABB.
    #[must_use]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Expand this AABB by a margin on all sides.
    #[must_use]
    pub fn expanded(&self, margin: f64) -> Self {
        Self {
            min: Point3::new(
                self.min.x - margin,
                self.min.y - margin,
                self.min.z - margin,
            ),
            max: Point3::new(
                self.max.x + margin,
                self.max.y + margin,
                self.max.z + margin,
            ),
        }
    }

    /// Get the extent (size) along a specific axis.
    #[must_use]
    pub fn extent(&self, axis: Axis) -> f64 {
        match axis {
            Axis::X => self.max.x - self.min.x,
            Axis::Y => self.max.y - self.min.y,
            Axis::Z => self.max.z - self.min.z,
        }
    }

    /// Get the minimum value along a specific axis.
    #[must_use]
    pub fn min_on_axis(&self, axis: Axis) -> f64 {
        match axis {
            Axis::X => self.min.x,
            Axis::Y => self.min.y,
            Axis::Z => self.min.z,
        }
    }

    /// Get the maximum value along a specific axis.
    #[must_use]
    pub fn max_on_axis(&self, axis: Axis) -> f64 {
        match axis {
            Axis::X => self.max.x,
            Axis::Y => self.max.y,
            Axis::Z => self.max.z,
        }
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Self::new(Point3::origin(), Point3::origin())
    }
}

/// Coordinate axis for sweep direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    /// X-axis (left-right).
    X,
    /// Y-axis (front-back).
    Y,
    /// Z-axis (up-down).
    Z,
}

impl Axis {
    /// Get all three axes.
    #[must_use]
    pub const fn all() -> [Self; 3] {
        [Self::X, Self::Y, Self::Z]
    }
}

/// Trait for broad-phase collision detection algorithms.
pub trait BroadPhase {
    /// Find all pairs of bodies that potentially collide.
    ///
    /// Returns pairs of body indices that have overlapping AABBs.
    /// The narrow phase should then check these pairs for actual collision.
    fn find_potential_pairs(&mut self, bodies: &[Body]) -> Vec<(BodyId, BodyId)>;
}

/// Sweep-and-Prune (Sort-and-Sweep) broad-phase algorithm.
///
/// This is an efficient O(n log n) algorithm that works by:
/// 1. Projecting body AABBs onto a chosen axis
/// 2. Sorting by minimum endpoint
/// 3. Sweeping to find overlapping intervals
///
/// For temporal coherence (bodies moving slowly between frames),
/// insertion sort can achieve O(n) complexity on nearly-sorted data.
#[derive(Debug, Clone)]
pub struct SweepAndPrune {
    /// Cached sorted list of `(body_index, min, max, body_id)` on the sweep axis.
    intervals: Vec<Interval>,
    /// The axis to sweep along (auto-selected based on scene extent).
    sweep_axis: Axis,
    /// Margin to add to AABBs for predictive collision detection.
    margin: f64,
}

/// An interval on the sweep axis.
#[derive(Debug, Clone, Copy)]
struct Interval {
    /// Index into the bodies array.
    body_index: usize,
    /// Body ID for the result.
    body_id: BodyId,
    /// Minimum endpoint on the sweep axis.
    min: f64,
    /// Maximum endpoint on the sweep axis.
    max: f64,
    /// Whether this body is static (for filtering static-static pairs).
    is_static: bool,
}

impl Default for SweepAndPrune {
    fn default() -> Self {
        Self::new()
    }
}

impl SweepAndPrune {
    /// Create a new sweep-and-prune broad phase.
    #[must_use]
    pub fn new() -> Self {
        Self {
            intervals: Vec::new(),
            sweep_axis: Axis::X,
            margin: 0.0,
        }
    }

    /// Create with a predictive margin for fast-moving objects.
    ///
    /// The margin expands AABBs to catch collisions before they happen.
    #[must_use]
    pub fn with_margin(mut self, margin: f64) -> Self {
        self.margin = margin;
        self
    }

    /// Choose the best axis to sweep based on scene variance.
    ///
    /// We pick the axis with the largest spread of body positions,
    /// which tends to minimize the number of overlapping intervals.
    fn choose_sweep_axis(bodies: &[Body]) -> Axis {
        if bodies.is_empty() {
            return Axis::X;
        }

        let mut min_pos = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max_pos = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

        for body in bodies {
            if body.collision_shape.is_none() {
                continue;
            }
            let pos = body.state.pose.position;
            min_pos.x = min_pos.x.min(pos.x);
            min_pos.y = min_pos.y.min(pos.y);
            min_pos.z = min_pos.z.min(pos.z);
            max_pos.x = max_pos.x.max(pos.x);
            max_pos.y = max_pos.y.max(pos.y);
            max_pos.z = max_pos.z.max(pos.z);
        }

        let extent_x = max_pos.x - min_pos.x;
        let extent_y = max_pos.y - min_pos.y;
        let extent_z = max_pos.z - min_pos.z;

        if extent_x >= extent_y && extent_x >= extent_z {
            Axis::X
        } else if extent_y >= extent_z {
            Axis::Y
        } else {
            Axis::Z
        }
    }

    /// Compute the AABB for a body.
    #[allow(clippy::too_many_lines)]
    fn compute_aabb(body: &Body) -> Option<Aabb> {
        let shape = body.collision_shape.as_ref()?;
        let pose = &body.state.pose;
        let center = pose.position;

        let aabb = match shape {
            CollisionShape::Sphere { radius } => {
                let half = Vector3::new(*radius, *radius, *radius);
                Aabb::from_center(center, half)
            }
            CollisionShape::Box { half_extents } => {
                // For a rotated box, we need to compute the world-space AABB
                // by transforming all 8 corners and finding min/max
                let rotation = pose.rotation.to_rotation_matrix();
                let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
                let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

                // Check all 8 corners
                for &sx in &[-1.0, 1.0] {
                    for &sy in &[-1.0, 1.0] {
                        for &sz in &[-1.0, 1.0] {
                            let local_corner = Vector3::new(
                                sx * half_extents.x,
                                sy * half_extents.y,
                                sz * half_extents.z,
                            );
                            let world_corner = center + rotation * local_corner;
                            min.x = min.x.min(world_corner.x);
                            min.y = min.y.min(world_corner.y);
                            min.z = min.z.min(world_corner.z);
                            max.x = max.x.max(world_corner.x);
                            max.y = max.y.max(world_corner.y);
                            max.z = max.z.max(world_corner.z);
                        }
                    }
                }

                Aabb::new(min, max)
            }
            CollisionShape::Capsule {
                half_length,
                radius,
            } => {
                // Capsule is oriented along local Z-axis
                let rotation = pose.rotation.to_rotation_matrix();
                let local_axis = Vector3::new(0.0, 0.0, *half_length);
                let world_axis = rotation * local_axis;

                // The AABB encompasses both endpoints plus radius
                let end1 = center + world_axis;
                let end2 = center - world_axis;

                Aabb::new(
                    Point3::new(
                        end1.x.min(end2.x) - radius,
                        end1.y.min(end2.y) - radius,
                        end1.z.min(end2.z) - radius,
                    ),
                    Point3::new(
                        end1.x.max(end2.x) + radius,
                        end1.y.max(end2.y) + radius,
                        end1.z.max(end2.z) + radius,
                    ),
                )
            }
            CollisionShape::Plane { normal, distance } => {
                // Planes are infinite, but we can represent them with a very large AABB
                // in the directions perpendicular to the normal
                const LARGE: f64 = 1e6;

                // Find a point on the plane
                let point_on_plane = Point3::from(*normal * *distance);

                // Create an AABB that's infinite in the plane's directions
                // but thin in the normal direction
                if normal.x.abs() > 0.9 {
                    // Plane roughly perpendicular to X
                    Aabb::new(
                        Point3::new(point_on_plane.x - 0.01, -LARGE, -LARGE),
                        Point3::new(point_on_plane.x + 0.01, LARGE, LARGE),
                    )
                } else if normal.y.abs() > 0.9 {
                    // Plane roughly perpendicular to Y
                    Aabb::new(
                        Point3::new(-LARGE, point_on_plane.y - 0.01, -LARGE),
                        Point3::new(LARGE, point_on_plane.y + 0.01, LARGE),
                    )
                } else {
                    // Plane roughly perpendicular to Z (most common: ground planes)
                    Aabb::new(
                        Point3::new(-LARGE, -LARGE, point_on_plane.z - 0.01),
                        Point3::new(LARGE, LARGE, point_on_plane.z + 0.01),
                    )
                }
            }
            CollisionShape::ConvexMesh { vertices } => {
                // Transform all vertices to world space and compute bounds
                let rotation = pose.rotation.to_rotation_matrix();
                let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
                let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

                for vertex in vertices {
                    let world_vertex = center + rotation * vertex.coords;
                    min.x = min.x.min(world_vertex.x);
                    min.y = min.y.min(world_vertex.y);
                    min.z = min.z.min(world_vertex.z);
                    max.x = max.x.max(world_vertex.x);
                    max.y = max.y.max(world_vertex.y);
                    max.z = max.z.max(world_vertex.z);
                }

                Aabb::new(min, max)
            }
            CollisionShape::Cylinder {
                half_length,
                radius,
            } => {
                // Cylinder is oriented along local Z-axis.
                // For a tight AABB, we need to compute the projection of the cylinder
                // onto each axis. The cylinder's AABB is the union of the two circular
                // end caps.
                let rotation = pose.rotation.to_rotation_matrix();

                // The cylinder axis in world space
                let axis = rotation * Vector3::new(0.0, 0.0, *half_length);

                // For each world axis, compute the extent contribution from:
                // 1. The cylinder axis (contributes |axis.i|)
                // 2. The circular cross-section perpendicular to the axis
                //    (contributes radius * sqrt(1 - (local_axis_dir.i)^2))
                let axis_z = Vector3::new(0.0, 0.0, 1.0);
                let world_z = rotation * axis_z; // Unit axis direction

                // Extent on each axis = |h*z_i| + r*sqrt(1 - z_i^2)
                // where z_i is the i-th component of the unit cylinder axis
                let extent_x = axis.x.abs() + radius * (1.0 - world_z.x.powi(2)).sqrt();
                let extent_y = axis.y.abs() + radius * (1.0 - world_z.y.powi(2)).sqrt();
                let extent_z = axis.z.abs() + radius * (1.0 - world_z.z.powi(2)).sqrt();

                Aabb::from_center(center, Vector3::new(extent_x, extent_y, extent_z))
            }
            CollisionShape::Ellipsoid { radii } => {
                // For a rotated ellipsoid, we compute the AABB by projecting
                // onto each axis. The extent along axis i is:
                // extent_i = sqrt(sum_j (R_ij * r_j)^2)
                // where R is the rotation matrix and r are the radii.
                let rotation = pose.rotation.to_rotation_matrix();

                let extent_x = (rotation[(0, 0)] * radii.x).powi(2)
                    + (rotation[(0, 1)] * radii.y).powi(2)
                    + (rotation[(0, 2)] * radii.z).powi(2);
                let extent_y = (rotation[(1, 0)] * radii.x).powi(2)
                    + (rotation[(1, 1)] * radii.y).powi(2)
                    + (rotation[(1, 2)] * radii.z).powi(2);
                let extent_z = (rotation[(2, 0)] * radii.x).powi(2)
                    + (rotation[(2, 1)] * radii.y).powi(2)
                    + (rotation[(2, 2)] * radii.z).powi(2);

                let half_extents = Vector3::new(extent_x.sqrt(), extent_y.sqrt(), extent_z.sqrt());
                Aabb::from_center(center, half_extents)
            }
            CollisionShape::HeightField { data } => {
                // Get local AABB of height field
                let (local_min, local_max) = data.aabb();

                // Transform to world space
                let rotation = pose.rotation.to_rotation_matrix();

                // Transform all 8 corners of the local AABB to world space
                let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
                let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

                for &lx in &[local_min.x, local_max.x] {
                    for &ly in &[local_min.y, local_max.y] {
                        for &lz in &[local_min.z, local_max.z] {
                            let local_corner = Vector3::new(lx, ly, lz);
                            let world_corner = center + rotation * local_corner;
                            min.x = min.x.min(world_corner.x);
                            min.y = min.y.min(world_corner.y);
                            min.z = min.z.min(world_corner.z);
                            max.x = max.x.max(world_corner.x);
                            max.y = max.y.max(world_corner.y);
                            max.z = max.z.max(world_corner.z);
                        }
                    }
                }

                Aabb::new(min, max)
            }
        };

        Some(aabb)
    }
}

impl BroadPhase for SweepAndPrune {
    fn find_potential_pairs(&mut self, bodies: &[Body]) -> Vec<(BodyId, BodyId)> {
        // Choose the sweep axis based on scene layout
        self.sweep_axis = Self::choose_sweep_axis(bodies);

        // Build intervals
        self.intervals.clear();
        for (index, body) in bodies.iter().enumerate() {
            // Skip bodies without collision shapes
            if body.collision_shape.is_none() {
                continue;
            }

            // Skip sleeping bodies
            if body.is_sleeping {
                continue;
            }

            if let Some(aabb) = Self::compute_aabb(body) {
                let aabb = if self.margin > 0.0 {
                    aabb.expanded(self.margin)
                } else {
                    aabb
                };

                self.intervals.push(Interval {
                    body_index: index,
                    body_id: body.id,
                    min: aabb.min_on_axis(self.sweep_axis),
                    max: aabb.max_on_axis(self.sweep_axis),
                    is_static: body.is_static,
                });
            }
        }

        // Sort by minimum endpoint
        // For nearly-sorted data (temporal coherence), insertion sort is O(n)
        // Rust's sort is adaptive and handles this well
        self.intervals.sort_by(|a, b| {
            a.min
                .partial_cmp(&b.min)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Sweep to find overlapping pairs
        let mut pairs = Vec::new();
        let n = self.intervals.len();

        for i in 0..n {
            let interval_i = &self.intervals[i];

            // Check all intervals that start before this one ends
            for j in (i + 1)..n {
                let interval_j = &self.intervals[j];

                // If j's min is past i's max, no more overlaps possible
                if interval_j.min > interval_i.max {
                    break;
                }

                // Skip static-static pairs (they don't collide meaningfully)
                if interval_i.is_static && interval_j.is_static {
                    continue;
                }

                // Check overlap on all three axes (not just sweep axis)
                // This is the key step that reduces false positives
                let body_i = &bodies[interval_i.body_index];
                let body_j = &bodies[interval_j.body_index];

                if let (Some(aabb_i), Some(aabb_j)) =
                    (Self::compute_aabb(body_i), Self::compute_aabb(body_j))
                {
                    let aabb_i = if self.margin > 0.0 {
                        aabb_i.expanded(self.margin)
                    } else {
                        aabb_i
                    };
                    let aabb_j = if self.margin > 0.0 {
                        aabb_j.expanded(self.margin)
                    } else {
                        aabb_j
                    };

                    if aabb_i.overlaps(&aabb_j) {
                        pairs.push((interval_i.body_id, interval_j.body_id));
                    }
                }
            }
        }

        pairs
    }
}

/// Simple O(n²) brute-force broad phase for comparison and small scenes.
///
/// This is the baseline algorithm that checks all pairs. It's suitable
/// for scenes with fewer than ~50 bodies where the overhead of more
/// sophisticated algorithms isn't worth it.
#[derive(Debug, Clone, Default)]
pub struct BruteForce {
    /// Margin for AABB expansion.
    margin: f64,
}

impl BruteForce {
    /// Create a new brute-force broad phase.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with a predictive margin.
    #[must_use]
    pub fn with_margin(mut self, margin: f64) -> Self {
        self.margin = margin;
        self
    }
}

impl BroadPhase for BruteForce {
    fn find_potential_pairs(&mut self, bodies: &[Body]) -> Vec<(BodyId, BodyId)> {
        let mut pairs = Vec::new();

        for (i, body_a) in bodies.iter().enumerate() {
            // Skip bodies without collision shapes or sleeping
            if body_a.collision_shape.is_none() || body_a.is_sleeping {
                continue;
            }

            for body_b in bodies.iter().skip(i + 1) {
                // Skip bodies without collision shapes or sleeping
                if body_b.collision_shape.is_none() || body_b.is_sleeping {
                    continue;
                }

                // Skip static-static pairs
                if body_a.is_static && body_b.is_static {
                    continue;
                }

                // Check AABB overlap
                if let (Some(aabb_a), Some(aabb_b)) = (
                    SweepAndPrune::compute_aabb(body_a),
                    SweepAndPrune::compute_aabb(body_b),
                ) {
                    let aabb_a = if self.margin > 0.0 {
                        aabb_a.expanded(self.margin)
                    } else {
                        aabb_a
                    };
                    let aabb_b = if self.margin > 0.0 {
                        aabb_b.expanded(self.margin)
                    } else {
                        aabb_b
                    };

                    if aabb_a.overlaps(&aabb_b) {
                        pairs.push((body_a.id, body_b.id));
                    }
                }
            }
        }

        pairs
    }
}

/// Configuration for broad-phase collision detection.
#[derive(Debug, Clone)]
pub struct BroadPhaseConfig {
    /// Algorithm to use for broad-phase detection.
    pub algorithm: BroadPhaseAlgorithm,
    /// Margin to add to AABBs for predictive detection.
    pub margin: f64,
    /// Threshold body count below which brute force is used.
    pub brute_force_threshold: usize,
}

impl Default for BroadPhaseConfig {
    fn default() -> Self {
        Self {
            algorithm: BroadPhaseAlgorithm::Auto,
            margin: 0.0,
            brute_force_threshold: 32,
        }
    }
}

/// Broad-phase algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BroadPhaseAlgorithm {
    /// Automatically choose based on body count.
    #[default]
    Auto,
    /// Always use brute force O(n²).
    BruteForce,
    /// Always use sweep-and-prune O(n log n).
    SweepAndPrune,
}

/// Manager for broad-phase collision detection.
///
/// This wraps the algorithm selection and provides a stable interface
/// for the World to use.
#[derive(Debug, Clone)]
pub struct BroadPhaseDetector {
    config: BroadPhaseConfig,
    sap: SweepAndPrune,
    brute: BruteForce,
}

impl Default for BroadPhaseDetector {
    fn default() -> Self {
        Self::new(BroadPhaseConfig::default())
    }
}

impl BroadPhaseDetector {
    /// Create a new broad-phase detector with the given configuration.
    #[must_use]
    pub fn new(config: BroadPhaseConfig) -> Self {
        Self {
            sap: SweepAndPrune::new().with_margin(config.margin),
            brute: BruteForce::new().with_margin(config.margin),
            config,
        }
    }

    /// Find all potentially colliding pairs.
    pub fn find_potential_pairs(&mut self, bodies: &[Body]) -> Vec<(BodyId, BodyId)> {
        match self.config.algorithm {
            BroadPhaseAlgorithm::Auto => {
                if bodies.len() < self.config.brute_force_threshold {
                    self.brute.find_potential_pairs(bodies)
                } else {
                    self.sap.find_potential_pairs(bodies)
                }
            }
            BroadPhaseAlgorithm::BruteForce => self.brute.find_potential_pairs(bodies),
            BroadPhaseAlgorithm::SweepAndPrune => self.sap.find_potential_pairs(bodies),
        }
    }

    /// Get the current configuration.
    #[must_use]
    pub fn config(&self) -> &BroadPhaseConfig {
        &self.config
    }

    /// Update the configuration.
    pub fn set_config(&mut self, config: BroadPhaseConfig) {
        self.sap = SweepAndPrune::new().with_margin(config.margin);
        self.brute = BruteForce::new().with_margin(config.margin);
        self.config = config;
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;
    use sim_types::{MassProperties, Pose, RigidBodyState};

    fn make_sphere_body(id: u64, pos: Point3<f64>, radius: f64) -> Body {
        Body::new(
            BodyId::new(id),
            RigidBodyState::at_rest(Pose::from_position(pos)),
            MassProperties::sphere(1.0, radius),
        )
        .with_collision_shape(CollisionShape::sphere(radius))
    }

    fn make_static_ground(id: u64) -> Body {
        Body::new_static(BodyId::new(id), Pose::identity())
            .with_collision_shape(CollisionShape::ground_plane(0.0))
    }

    #[test]
    fn test_aabb_overlaps() {
        let a = Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0));
        let b = Aabb::from_center(Point3::new(1.5, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));
        let c = Aabb::from_center(Point3::new(5.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0));

        assert!(a.overlaps(&b), "a and b should overlap");
        assert!(b.overlaps(&a), "overlap should be symmetric");
        assert!(!a.overlaps(&c), "a and c should not overlap");
    }

    #[test]
    fn test_aabb_expanded() {
        let aabb = Aabb::from_center(Point3::origin(), Vector3::new(1.0, 1.0, 1.0));
        let expanded = aabb.expanded(0.5);

        assert_eq!(expanded.min.x, -1.5);
        assert_eq!(expanded.max.x, 1.5);
    }

    #[test]
    fn test_sweep_and_prune_finds_overlapping_spheres() {
        let bodies = vec![
            make_sphere_body(1, Point3::new(0.0, 0.0, 0.0), 1.0),
            make_sphere_body(2, Point3::new(1.5, 0.0, 0.0), 1.0),
        ];

        let mut sap = SweepAndPrune::new();
        let pairs = sap.find_potential_pairs(&bodies);

        assert_eq!(pairs.len(), 1);
        assert!(pairs.contains(&(BodyId::new(1), BodyId::new(2))));
    }

    #[test]
    fn test_sweep_and_prune_no_overlap() {
        let bodies = vec![
            make_sphere_body(1, Point3::new(0.0, 0.0, 0.0), 1.0),
            make_sphere_body(2, Point3::new(5.0, 0.0, 0.0), 1.0),
        ];

        let mut sap = SweepAndPrune::new();
        let pairs = sap.find_potential_pairs(&bodies);

        assert!(pairs.is_empty());
    }

    #[test]
    fn test_sweep_and_prune_skips_static_static() {
        let bodies = vec![make_static_ground(1), make_static_ground(2)];

        let mut sap = SweepAndPrune::new();
        let pairs = sap.find_potential_pairs(&bodies);

        assert!(pairs.is_empty(), "static-static pairs should be skipped");
    }

    #[test]
    fn test_sweep_and_prune_includes_static_dynamic() {
        let bodies = vec![
            make_static_ground(1),
            make_sphere_body(2, Point3::new(0.0, 0.0, 0.5), 1.0),
        ];

        let mut sap = SweepAndPrune::new();
        let pairs = sap.find_potential_pairs(&bodies);

        assert_eq!(pairs.len(), 1, "static-dynamic pairs should be included");
    }

    #[test]
    fn test_brute_force_matches_sap() {
        let bodies = vec![
            make_sphere_body(1, Point3::new(0.0, 0.0, 0.0), 1.0),
            make_sphere_body(2, Point3::new(1.5, 0.0, 0.0), 1.0),
            make_sphere_body(3, Point3::new(0.0, 1.5, 0.0), 1.0),
            make_sphere_body(4, Point3::new(5.0, 0.0, 0.0), 1.0),
        ];

        let mut sap = SweepAndPrune::new();
        let mut brute = BruteForce::new();

        let sap_pairs = sap.find_potential_pairs(&bodies);
        let brute_pairs = brute.find_potential_pairs(&bodies);

        // Both should find the same pairs (order may differ, and (a,b) vs (b,a) may differ)
        assert_eq!(sap_pairs.len(), brute_pairs.len());

        // Helper to normalize pair order for comparison
        let normalize = |pair: &(BodyId, BodyId)| {
            if pair.0.raw() < pair.1.raw() {
                *pair
            } else {
                (pair.1, pair.0)
            }
        };

        let sap_normalized: std::collections::HashSet<_> =
            sap_pairs.iter().map(normalize).collect();
        let brute_normalized: std::collections::HashSet<_> =
            brute_pairs.iter().map(normalize).collect();

        assert_eq!(sap_normalized, brute_normalized);
    }

    #[test]
    fn test_broad_phase_detector_auto() {
        let mut detector = BroadPhaseDetector::default();

        // Small scene uses brute force
        let small_bodies: Vec<_> = (0..10)
            .map(|i| make_sphere_body(i, Point3::new(i as f64 * 3.0, 0.0, 0.0), 1.0))
            .collect();
        let _ = detector.find_potential_pairs(&small_bodies);

        // Large scene uses SAP
        let large_bodies: Vec<_> = (0..100)
            .map(|i| make_sphere_body(i, Point3::new(i as f64 * 3.0, 0.0, 0.0), 1.0))
            .collect();
        let _ = detector.find_potential_pairs(&large_bodies);
    }

    #[test]
    fn test_compute_aabb_sphere() {
        let body = make_sphere_body(1, Point3::new(5.0, 5.0, 5.0), 2.0);
        let aabb = SweepAndPrune::compute_aabb(&body).unwrap();

        assert_eq!(aabb.min, Point3::new(3.0, 3.0, 3.0));
        assert_eq!(aabb.max, Point3::new(7.0, 7.0, 7.0));
    }

    #[test]
    fn test_compute_aabb_box() {
        let body = Body::new(
            BodyId::new(1),
            RigidBodyState::at_rest(Pose::from_position(Point3::origin())),
            MassProperties::box_shape(1.0, Vector3::new(1.0, 2.0, 3.0)),
        )
        .with_collision_shape(CollisionShape::box_shape(Vector3::new(1.0, 2.0, 3.0)));

        let aabb = SweepAndPrune::compute_aabb(&body).unwrap();

        assert_eq!(aabb.min, Point3::new(-1.0, -2.0, -3.0));
        assert_eq!(aabb.max, Point3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn test_skips_sleeping_bodies() {
        let mut body = make_sphere_body(1, Point3::origin(), 1.0);
        body.is_sleeping = true;

        let bodies = vec![body, make_sphere_body(2, Point3::new(0.5, 0.0, 0.0), 1.0)];

        let mut sap = SweepAndPrune::new();
        let pairs = sap.find_potential_pairs(&bodies);

        assert!(pairs.is_empty(), "sleeping bodies should be skipped");
    }

    #[test]
    fn test_skips_bodies_without_collision_shape() {
        let body_no_shape = Body::new(
            BodyId::new(1),
            RigidBodyState::at_rest(Pose::from_position(Point3::origin())),
            MassProperties::sphere(1.0, 1.0),
        );
        // Note: no collision shape added

        let bodies = vec![
            body_no_shape,
            make_sphere_body(2, Point3::new(0.5, 0.0, 0.0), 1.0),
        ];

        let mut sap = SweepAndPrune::new();
        let pairs = sap.find_potential_pairs(&bodies);

        assert!(
            pairs.is_empty(),
            "bodies without collision shapes should be skipped"
        );
    }

    #[test]
    fn test_margin_expands_detection() {
        // Two spheres just barely not touching
        let bodies = vec![
            make_sphere_body(1, Point3::new(0.0, 0.0, 0.0), 1.0),
            make_sphere_body(2, Point3::new(2.1, 0.0, 0.0), 1.0),
        ];

        // Without margin, they shouldn't overlap
        let mut sap_no_margin = SweepAndPrune::new();
        let pairs_no_margin = sap_no_margin.find_potential_pairs(&bodies);
        assert!(pairs_no_margin.is_empty());

        // With margin, they should overlap
        let mut sap_with_margin = SweepAndPrune::new().with_margin(0.2);
        let pairs_with_margin = sap_with_margin.find_potential_pairs(&bodies);
        assert_eq!(pairs_with_margin.len(), 1);
    }
}
