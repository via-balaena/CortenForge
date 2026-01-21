//! Tendon path geometry with via points and segments.
//!
//! This module provides types for representing the 3D path of a tendon
//! through the body. Tendons attach at origin and insertion points and
//! may pass through intermediate via points.
//!
//! # Path Computation
//!
//! The total tendon length is computed as the sum of segment lengths:
//!
//! ```text
//! L = Σᵢ ||pᵢ₊₁ - pᵢ||
//! ```
//!
//! Where each point `pᵢ` is transformed from its local body frame to world
//! coordinates using the body's current pose.

use nalgebra::{Isometry3, Point3, Vector3};
use sim_types::BodyId;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// An attachment point on a body.
///
/// Attachment points define where a tendon connects to or passes through
/// a body. The position is specified in the body's local frame.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AttachmentPoint {
    /// The body this point is attached to.
    pub body: BodyId,

    /// Position in the body's local frame.
    pub local_position: Point3<f64>,

    /// Optional local tangent direction for smooth routing.
    ///
    /// When specified, the tendon should approach this point along
    /// this direction. Used for wrapping calculations.
    pub tangent: Option<Vector3<f64>>,
}

impl AttachmentPoint {
    /// Create a new attachment point.
    #[must_use]
    pub fn new(body: BodyId, local_position: Point3<f64>) -> Self {
        Self {
            body,
            local_position,
            tangent: None,
        }
    }

    /// Create an attachment point at the body origin.
    #[must_use]
    pub fn at_origin(body: BodyId) -> Self {
        Self::new(body, Point3::origin())
    }

    /// Set the tangent direction.
    #[must_use]
    pub fn with_tangent(mut self, tangent: Vector3<f64>) -> Self {
        self.tangent = Some(tangent.normalize());
        self
    }

    /// Transform the point to world coordinates.
    ///
    /// # Arguments
    ///
    /// * `body_transform` - The body's pose (world from body transform)
    ///
    /// # Returns
    ///
    /// The attachment point in world coordinates.
    #[must_use]
    pub fn world_position(&self, body_transform: &Isometry3<f64>) -> Point3<f64> {
        body_transform * self.local_position
    }

    /// Transform the tangent to world coordinates.
    #[must_use]
    pub fn world_tangent(&self, body_transform: &Isometry3<f64>) -> Option<Vector3<f64>> {
        self.tangent.map(|t| body_transform.rotation * t)
    }
}

/// A segment of the tendon path between two points.
///
/// Segments store cached information about the path for efficient force
/// computation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TendonSegment {
    /// Start point index in the path.
    pub start_index: usize,

    /// End point index in the path.
    pub end_index: usize,

    /// Cached segment length (updated when bodies move).
    pub length: f64,

    /// Cached unit direction vector (from start to end).
    pub direction: Vector3<f64>,
}

impl TendonSegment {
    /// Create a new segment between two points.
    #[must_use]
    pub fn new(start_index: usize, end_index: usize) -> Self {
        Self {
            start_index,
            end_index,
            length: 0.0,
            direction: Vector3::zeros(),
        }
    }

    /// Update the segment from world-space point positions.
    pub fn update(&mut self, start: &Point3<f64>, end: &Point3<f64>) {
        let delta = end - start;
        self.length = delta.norm();

        if self.length > 1e-10 {
            self.direction = delta / self.length;
        } else {
            self.direction = Vector3::zeros();
        }
    }
}

/// The complete path of a tendon through bodies.
///
/// A tendon path consists of:
/// - An origin attachment (where the tendon starts)
/// - Zero or more via points (intermediate routing points)
/// - An insertion attachment (where the tendon ends)
///
/// # Example
///
/// ```
/// use sim_tendon::path::{TendonPath, AttachmentPoint};
/// use sim_types::BodyId;
/// use nalgebra::Point3;
///
/// let path = TendonPath::new(
///     AttachmentPoint::new(BodyId::new(0), Point3::new(0.0, 0.0, 0.1)),
///     AttachmentPoint::new(BodyId::new(1), Point3::new(0.0, 0.0, -0.1)),
/// )
/// .with_via_point(AttachmentPoint::new(BodyId::new(0), Point3::new(0.05, 0.0, 0.0)));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TendonPath {
    /// The origin attachment point.
    pub origin: AttachmentPoint,

    /// Intermediate via points.
    pub via_points: Vec<AttachmentPoint>,

    /// The insertion attachment point.
    pub insertion: AttachmentPoint,

    /// Cached segments (updated when computing length).
    segments: Vec<TendonSegment>,

    /// Cached total length.
    cached_length: f64,
}

impl TendonPath {
    /// Create a new tendon path from origin to insertion.
    #[must_use]
    pub fn new(origin: AttachmentPoint, insertion: AttachmentPoint) -> Self {
        let segments = vec![TendonSegment::new(0, 1)];

        Self {
            origin,
            via_points: Vec::new(),
            insertion,
            segments,
            cached_length: 0.0,
        }
    }

    /// Create a straight tendon path between two bodies.
    #[must_use]
    pub fn straight(
        origin_body: BodyId,
        origin_pos: Point3<f64>,
        insertion_body: BodyId,
        insertion_pos: Point3<f64>,
    ) -> Self {
        Self::new(
            AttachmentPoint::new(origin_body, origin_pos),
            AttachmentPoint::new(insertion_body, insertion_pos),
        )
    }

    /// Add a via point to the path.
    #[must_use]
    pub fn with_via_point(mut self, via_point: AttachmentPoint) -> Self {
        self.via_points.push(via_point);
        self.rebuild_segments();
        self
    }

    /// Add multiple via points to the path.
    #[must_use]
    pub fn with_via_points(
        mut self,
        via_points: impl IntoIterator<Item = AttachmentPoint>,
    ) -> Self {
        self.via_points.extend(via_points);
        self.rebuild_segments();
        self
    }

    /// Get the total number of points in the path.
    #[must_use]
    pub fn num_points(&self) -> usize {
        2 + self.via_points.len()
    }

    /// Get the number of segments in the path.
    #[must_use]
    pub fn num_segments(&self) -> usize {
        self.segments.len()
    }

    /// Get a point by index.
    ///
    /// Index 0 is origin, indices 1..n-1 are via points, index n is insertion.
    #[must_use]
    pub fn point(&self, index: usize) -> Option<&AttachmentPoint> {
        let n = self.num_points();
        if index >= n {
            return None;
        }

        if index == 0 {
            Some(&self.origin)
        } else if index == n - 1 {
            Some(&self.insertion)
        } else {
            self.via_points.get(index - 1)
        }
    }

    /// Get all bodies that this tendon attaches to.
    #[must_use]
    pub fn bodies(&self) -> Vec<BodyId> {
        let mut bodies = Vec::with_capacity(self.num_points());
        bodies.push(self.origin.body);

        for via in &self.via_points {
            if !bodies.contains(&via.body) {
                bodies.push(via.body);
            }
        }

        if !bodies.contains(&self.insertion.body) {
            bodies.push(self.insertion.body);
        }

        bodies
    }

    /// Rebuild segments after modifying via points.
    fn rebuild_segments(&mut self) {
        let n = self.num_points();
        self.segments.clear();

        for i in 0..n - 1 {
            self.segments.push(TendonSegment::new(i, i + 1));
        }
    }

    /// Compute the total path length given body transforms.
    ///
    /// # Arguments
    ///
    /// * `get_transform` - Function to get body transform by ID
    ///
    /// # Returns
    ///
    /// Total tendon length in meters.
    pub fn compute_length<F>(&mut self, get_transform: F) -> f64
    where
        F: Fn(BodyId) -> Isometry3<f64>,
    {
        // Compute world positions of all points
        let mut world_points = Vec::with_capacity(self.num_points());

        world_points.push(self.origin.world_position(&get_transform(self.origin.body)));

        for via in &self.via_points {
            world_points.push(via.world_position(&get_transform(via.body)));
        }

        world_points.push(
            self.insertion
                .world_position(&get_transform(self.insertion.body)),
        );

        // Update segments and compute total length
        let mut total_length = 0.0;

        for segment in &mut self.segments {
            segment.update(
                &world_points[segment.start_index],
                &world_points[segment.end_index],
            );
            total_length += segment.length;
        }

        self.cached_length = total_length;
        total_length
    }

    /// Get the cached total length.
    ///
    /// Call `compute_length` first to update this value.
    #[must_use]
    pub fn cached_length(&self) -> f64 {
        self.cached_length
    }

    /// Get the segments.
    #[must_use]
    pub fn segments(&self) -> &[TendonSegment] {
        &self.segments
    }

    /// Compute the path velocity given body velocities.
    ///
    /// # Arguments
    ///
    /// * `get_transform` - Function to get body transform by ID
    /// * `get_velocity` - Function to get body velocity (linear, angular) by ID
    ///
    /// # Returns
    ///
    /// Rate of length change in m/s (positive = lengthening).
    pub fn compute_velocity<F, G>(&self, get_transform: F, get_velocity: G) -> f64
    where
        F: Fn(BodyId) -> Isometry3<f64>,
        G: Fn(BodyId) -> (Vector3<f64>, Vector3<f64>),
    {
        // Velocity is computed as dL/dt = Σ (direction · velocity_of_point)
        // For each segment endpoint

        let mut world_points = Vec::with_capacity(self.num_points());
        let mut world_velocities = Vec::with_capacity(self.num_points());

        // Origin
        let origin_transform = get_transform(self.origin.body);
        let (origin_linear, origin_angular) = get_velocity(self.origin.body);
        let origin_world = origin_transform * self.origin.local_position;
        let origin_offset = origin_world.coords - origin_transform.translation.vector;
        let origin_vel = origin_linear + origin_angular.cross(&origin_offset);

        world_points.push(origin_world);
        world_velocities.push(origin_vel);

        // Via points
        for via in &self.via_points {
            let transform = get_transform(via.body);
            let (linear, angular) = get_velocity(via.body);
            let world = transform * via.local_position;
            let offset = world.coords - transform.translation.vector;
            let vel = linear + angular.cross(&offset);

            world_points.push(world);
            world_velocities.push(vel);
        }

        // Insertion
        let insertion_transform = get_transform(self.insertion.body);
        let (insertion_linear, insertion_angular) = get_velocity(self.insertion.body);
        let insertion_world = insertion_transform * self.insertion.local_position;
        let insertion_offset = insertion_world.coords - insertion_transform.translation.vector;
        let insertion_vel = insertion_linear + insertion_angular.cross(&insertion_offset);

        world_points.push(insertion_world);
        world_velocities.push(insertion_vel);

        // Compute velocity contribution from each segment
        let mut total_velocity = 0.0;

        for segment in &self.segments {
            let p_start = &world_points[segment.start_index];
            let p_end = &world_points[segment.end_index];
            let v_start = &world_velocities[segment.start_index];
            let v_end = &world_velocities[segment.end_index];

            let delta = p_end - p_start;
            let length = delta.norm();

            if length > 1e-10 {
                let direction = delta / length;
                // Rate of length change = (v_end - v_start) · direction
                let rel_vel = v_end - v_start;
                total_velocity += direction.dot(&rel_vel);
            }
        }

        total_velocity
    }

    /// Compute forces on bodies from tendon tension.
    ///
    /// # Arguments
    ///
    /// * `tension` - Tendon tension in Newtons
    /// * `get_transform` - Function to get body transform by ID
    ///
    /// # Returns
    ///
    /// Vector of (body_id, force, torque) tuples representing forces
    /// applied to each body.
    #[must_use]
    pub fn compute_forces<F>(
        &self,
        tension: f64,
        get_transform: F,
    ) -> Vec<(BodyId, Vector3<f64>, Vector3<f64>)>
    where
        F: Fn(BodyId) -> Isometry3<f64>,
    {
        if tension.abs() < 1e-10 {
            return Vec::new();
        }

        // Compute world positions
        let mut world_points = Vec::with_capacity(self.num_points());

        world_points.push(self.origin.world_position(&get_transform(self.origin.body)));

        for via in &self.via_points {
            world_points.push(via.world_position(&get_transform(via.body)));
        }

        world_points.push(
            self.insertion
                .world_position(&get_transform(self.insertion.body)),
        );

        // Compute segment directions
        let mut directions: Vec<Vector3<f64>> = Vec::with_capacity(self.num_segments());
        for i in 0..self.num_segments() {
            let delta = world_points[i + 1] - world_points[i];
            let length = delta.norm();
            if length > 1e-10 {
                directions.push(delta / length);
            } else {
                directions.push(Vector3::zeros());
            }
        }

        // Compute forces at each point
        // Force at origin: along first segment direction
        // Force at insertion: opposite of last segment direction
        // Force at via point: change in direction

        let mut forces = Vec::new();

        // Origin force
        let origin_force = -tension * directions[0]; // Tendon pulls toward first via/insertion
        let origin_transform = get_transform(self.origin.body);
        let origin_world = world_points[0];
        let origin_com = Point3::from(origin_transform.translation.vector);
        let origin_torque = (origin_world - origin_com).cross(&origin_force);
        forces.push((self.origin.body, origin_force, origin_torque));

        // Via point forces (change in direction)
        for (i, via) in self.via_points.iter().enumerate() {
            let dir_in = directions[i];
            let dir_out = directions[i + 1];
            let via_force = tension * (dir_in - dir_out); // Net change in direction

            let via_transform = get_transform(via.body);
            let via_world = world_points[i + 1];
            let via_com = Point3::from(via_transform.translation.vector);
            let via_torque = (via_world - via_com).cross(&via_force);
            forces.push((via.body, via_force, via_torque));
        }

        // Insertion force
        let last_idx = directions.len() - 1;
        let insertion_force = tension * directions[last_idx]; // Tendon pulls toward last via/origin
        let insertion_transform = get_transform(self.insertion.body);
        let insertion_world = *world_points.last().unwrap_or(&Point3::origin());
        let insertion_com = Point3::from(insertion_transform.translation.vector);
        let insertion_torque = (insertion_world - insertion_com).cross(&insertion_force);
        forces.push((self.insertion.body, insertion_force, insertion_torque));

        forces
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn identity_transform(_: BodyId) -> Isometry3<f64> {
        Isometry3::identity()
    }

    fn zero_velocity(_: BodyId) -> (Vector3<f64>, Vector3<f64>) {
        (Vector3::zeros(), Vector3::zeros())
    }

    #[test]
    fn test_attachment_point() {
        let point = AttachmentPoint::new(BodyId::new(0), Point3::new(1.0, 0.0, 0.0));

        let transform = Isometry3::translation(10.0, 0.0, 0.0);
        let world = point.world_position(&transform);

        assert_relative_eq!(world.x, 11.0, epsilon = 1e-10);
    }

    #[test]
    fn test_straight_path_length() {
        let mut path = TendonPath::straight(
            BodyId::new(0),
            Point3::new(0.0, 0.0, 0.0),
            BodyId::new(1),
            Point3::new(1.0, 0.0, 0.0),
        );

        let length = path.compute_length(identity_transform);
        assert_relative_eq!(length, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_path_with_via_point() {
        let mut path = TendonPath::straight(
            BodyId::new(0),
            Point3::new(0.0, 0.0, 0.0),
            BodyId::new(2),
            Point3::new(2.0, 0.0, 0.0),
        )
        .with_via_point(AttachmentPoint::new(
            BodyId::new(1),
            Point3::new(1.0, 1.0, 0.0),
        ));

        let length = path.compute_length(identity_transform);
        // Distance: sqrt(2) + sqrt(2) = 2*sqrt(2) ≈ 2.828
        let expected = 2.0 * std::f64::consts::SQRT_2;
        assert_relative_eq!(length, expected, epsilon = 1e-10);
    }

    #[test]
    fn test_path_velocity_stationary() {
        let mut path = TendonPath::straight(
            BodyId::new(0),
            Point3::origin(),
            BodyId::new(1),
            Point3::new(1.0, 0.0, 0.0),
        );

        // Update length first
        let _ = path.compute_length(identity_transform);

        let velocity = path.compute_velocity(identity_transform, zero_velocity);
        assert_relative_eq!(velocity, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_path_velocity_stretching() {
        let mut path = TendonPath::straight(
            BodyId::new(0),
            Point3::origin(),
            BodyId::new(1),
            Point3::new(1.0, 0.0, 0.0),
        );

        let _ = path.compute_length(identity_transform);

        // Body 1 moving away at 1 m/s
        let velocity = path.compute_velocity(identity_transform, |id| {
            if id == BodyId::new(1) {
                (Vector3::new(1.0, 0.0, 0.0), Vector3::zeros())
            } else {
                (Vector3::zeros(), Vector3::zeros())
            }
        });

        assert_relative_eq!(velocity, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_forces_opposite_at_endpoints() {
        let mut path = TendonPath::straight(
            BodyId::new(0),
            Point3::origin(),
            BodyId::new(1),
            Point3::new(1.0, 0.0, 0.0),
        );

        let _ = path.compute_length(identity_transform);
        let forces = path.compute_forces(100.0, identity_transform);

        assert_eq!(forces.len(), 2);

        // Forces should be equal and opposite
        let (_, f0, _) = forces[0];
        let (_, f1, _) = forces[1];

        assert_relative_eq!((f0 + f1).norm(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_bodies() {
        let path = TendonPath::straight(
            BodyId::new(0),
            Point3::origin(),
            BodyId::new(1),
            Point3::new(1.0, 0.0, 0.0),
        );

        let bodies = path.bodies();
        assert_eq!(bodies.len(), 2);
        assert!(bodies.contains(&BodyId::new(0)));
        assert!(bodies.contains(&BodyId::new(1)));
    }
}
