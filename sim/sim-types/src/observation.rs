//! Observation types for simulation output.
//!
//! Observations are what the simulation produces: body states, sensor readings,
//! contact information, etc. This is the data that policies/controllers consume.

use crate::{BodyId, JointId, JointState, Pose, RigidBodyState, Twist};
use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// An observation from the simulation.
///
/// Contains the state of bodies, joints, contacts, and any other information
/// that a controller might need.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Observation {
    /// Simulation time at which this observation was taken.
    pub time: f64,
    /// Observations by type.
    pub data: Vec<ObservationType>,
}

impl Observation {
    /// Create a new observation at the given time.
    #[must_use]
    pub fn new(time: f64) -> Self {
        Self {
            time,
            data: Vec::new(),
        }
    }

    /// Add an observation type.
    pub fn add(&mut self, obs: ObservationType) {
        self.data.push(obs);
    }

    /// Create with body states.
    #[must_use]
    pub fn with_bodies(time: f64, bodies: Vec<(BodyId, RigidBodyState)>) -> Self {
        Self {
            time,
            data: vec![ObservationType::BodyStates(bodies)],
        }
    }

    /// Get body states if present.
    #[must_use]
    pub fn body_states(&self) -> Option<&[(BodyId, RigidBodyState)]> {
        for obs in &self.data {
            if let ObservationType::BodyStates(states) = obs {
                return Some(states);
            }
        }
        None
    }

    /// Get joint states if present.
    #[must_use]
    pub fn joint_states(&self) -> Option<&[(JointId, JointState)]> {
        for obs in &self.data {
            if let ObservationType::JointStates(states) = obs {
                return Some(states);
            }
        }
        None
    }

    /// Get contacts if present.
    #[must_use]
    pub fn contacts(&self) -> Option<&[ContactInfo]> {
        for obs in &self.data {
            if let ObservationType::Contacts(contacts) = obs {
                return Some(contacts);
            }
        }
        None
    }

    /// Check if there are any contacts.
    #[must_use]
    pub fn has_contacts(&self) -> bool {
        self.contacts().is_some_and(|c| !c.is_empty())
    }

    /// Get the state of a specific body.
    #[must_use]
    pub fn body_state(&self, id: BodyId) -> Option<&RigidBodyState> {
        self.body_states()
            .and_then(|states| states.iter().find(|(bid, _)| *bid == id).map(|(_, s)| s))
    }

    /// Get the state of a specific joint.
    #[must_use]
    pub fn joint_state(&self, id: JointId) -> Option<&JointState> {
        self.joint_states()
            .and_then(|states| states.iter().find(|(jid, _)| *jid == id).map(|(_, s)| s))
    }
}

/// Types of observations that can be included.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ObservationType {
    /// States of rigid bodies.
    BodyStates(Vec<(BodyId, RigidBodyState)>),

    /// States of joints.
    JointStates(Vec<(JointId, JointState)>),

    /// Contact information.
    Contacts(Vec<ContactInfo>),

    /// Center of mass of the entire system.
    SystemCom(Point3<f64>),

    /// Total momentum of the system.
    SystemMomentum {
        /// Linear momentum.
        linear: Vector3<f64>,
        /// Angular momentum about origin.
        angular: Vector3<f64>,
    },

    /// Total energy of the system.
    SystemEnergy {
        /// Kinetic energy.
        kinetic: f64,
        /// Potential energy (e.g., gravitational).
        potential: f64,
    },

    /// Custom named data (for extensibility).
    Custom {
        /// Name of the custom observation.
        name: String,
        /// Data as a flat array of f64 values.
        data: Vec<f64>,
    },
}

/// Information about a contact point.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ContactInfo {
    /// First body in contact.
    pub body_a: BodyId,
    /// Second body in contact (or None for static environment).
    pub body_b: Option<BodyId>,
    /// Contact point in world coordinates.
    pub position: Point3<f64>,
    /// Contact normal (pointing from A to B).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive means overlapping).
    pub depth: f64,
    /// Normal impulse applied to resolve contact.
    pub impulse_normal: f64,
    /// Tangential impulse (friction).
    pub impulse_tangent: Vector3<f64>,
}

impl ContactInfo {
    /// Create a new contact info.
    #[must_use]
    pub fn new(
        body_a: BodyId,
        body_b: Option<BodyId>,
        position: Point3<f64>,
        normal: Vector3<f64>,
        depth: f64,
    ) -> Self {
        Self {
            body_a,
            body_b,
            position,
            normal,
            depth,
            impulse_normal: 0.0,
            impulse_tangent: Vector3::zeros(),
        }
    }

    /// Create a contact with the static environment.
    #[must_use]
    pub fn with_ground(
        body: BodyId,
        position: Point3<f64>,
        normal: Vector3<f64>,
        depth: f64,
    ) -> Self {
        Self::new(body, None, position, normal, depth)
    }

    /// Create a contact between two bodies.
    #[must_use]
    pub fn between_bodies(
        body_a: BodyId,
        body_b: BodyId,
        position: Point3<f64>,
        normal: Vector3<f64>,
        depth: f64,
    ) -> Self {
        Self::new(body_a, Some(body_b), position, normal, depth)
    }

    /// Set the impulses applied during contact resolution.
    #[must_use]
    pub fn with_impulses(mut self, normal: f64, tangent: Vector3<f64>) -> Self {
        self.impulse_normal = normal;
        self.impulse_tangent = tangent;
        self
    }

    /// Check if this contact involves the ground/environment.
    #[must_use]
    pub fn is_ground_contact(&self) -> bool {
        self.body_b.is_none()
    }

    /// Check if this contact involves a specific body.
    #[must_use]
    pub fn involves_body(&self, body: BodyId) -> bool {
        self.body_a == body || self.body_b == Some(body)
    }

    /// Get the total impulse magnitude.
    #[must_use]
    pub fn total_impulse(&self) -> f64 {
        (self.impulse_normal * self.impulse_normal + self.impulse_tangent.norm_squared()).sqrt()
    }

    /// Get the contact force (impulse / dt).
    #[must_use]
    pub fn force(&self, dt: f64) -> Vector3<f64> {
        if dt > 0.0 {
            (self.normal * self.impulse_normal + self.impulse_tangent) / dt
        } else {
            Vector3::zeros()
        }
    }
}

/// Pose observation for a single body (lighter weight than full state).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PoseObservation {
    /// Body ID.
    pub body: BodyId,
    /// Pose at observation time.
    pub pose: Pose,
}

impl PoseObservation {
    /// Create a new pose observation.
    #[must_use]
    pub fn new(body: BodyId, pose: Pose) -> Self {
        Self { body, pose }
    }
}

/// Velocity observation for a single body.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct VelocityObservation {
    /// Body ID.
    pub body: BodyId,
    /// Twist at observation time.
    pub twist: Twist,
}

impl VelocityObservation {
    /// Create a new velocity observation.
    #[must_use]
    pub fn new(body: BodyId, twist: Twist) -> Self {
        Self { body, twist }
    }
}

/// Aggregated contact statistics for a body.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ContactStats {
    /// Number of contacts.
    pub count: usize,
    /// Total normal force magnitude.
    pub total_normal_force: f64,
    /// Total tangential force magnitude.
    pub total_tangent_force: f64,
    /// Average contact position (if any contacts).
    pub average_position: Option<Point3<f64>>,
    /// Average contact normal (if any contacts).
    pub average_normal: Option<Vector3<f64>>,
}

impl ContactStats {
    /// Compute statistics from a list of contacts for a specific body.
    #[must_use]
    pub fn for_body(contacts: &[ContactInfo], body: BodyId, dt: f64) -> Self {
        let body_contacts: Vec<_> = contacts.iter().filter(|c| c.involves_body(body)).collect();

        if body_contacts.is_empty() {
            return Self::default();
        }

        let count = body_contacts.len();
        let mut total_normal = 0.0;
        let mut total_tangent = 0.0;
        let mut sum_position = Vector3::zeros();
        let mut sum_normal = Vector3::zeros();

        for c in &body_contacts {
            total_normal += c.impulse_normal.abs() / dt.max(1e-10);
            total_tangent += c.impulse_tangent.norm() / dt.max(1e-10);
            sum_position += c.position.coords;
            sum_normal += c.normal;
        }

        let n = count as f64;
        Self {
            count,
            total_normal_force: total_normal,
            total_tangent_force: total_tangent,
            average_position: Some(Point3::from(sum_position / n)),
            average_normal: Some((sum_normal / n).normalize()),
        }
    }

    /// Check if there are any contacts.
    #[must_use]
    pub fn has_contacts(&self) -> bool {
        self.count > 0
    }

    /// Get total contact force magnitude.
    #[must_use]
    pub fn total_force(&self) -> f64 {
        self.total_normal_force.hypot(self.total_tangent_force)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn test_observation_creation() {
        let obs = Observation::new(1.5);
        assert_eq!(obs.time, 1.5);
        assert!(obs.data.is_empty());
    }

    #[test]
    fn test_observation_with_bodies() {
        let body_id = BodyId::new(1);
        let state = RigidBodyState::origin();
        let obs = Observation::with_bodies(0.0, vec![(body_id, state)]);

        assert!(obs.body_states().is_some());
        assert_eq!(obs.body_states().map(|s| s.len()), Some(1));
    }

    #[test]
    fn test_observation_lookup() {
        let body_id = BodyId::new(42);
        let state = RigidBodyState::at_rest(Pose::from_position(Point3::new(1.0, 2.0, 3.0)));
        let obs = Observation::with_bodies(0.0, vec![(body_id, state)]);

        let found = obs.body_state(body_id);
        assert!(found.is_some());
        assert_eq!(found.map(|s| s.pose.position.x), Some(1.0));

        let not_found = obs.body_state(BodyId::new(999));
        assert!(not_found.is_none());
    }

    #[test]
    fn test_contact_info() {
        let body_a = BodyId::new(1);
        let body_b = BodyId::new(2);
        let contact = ContactInfo::between_bodies(
            body_a,
            body_b,
            Point3::new(0.0, 0.0, 0.0),
            Vector3::z(),
            0.01,
        );

        assert!(contact.involves_body(body_a));
        assert!(contact.involves_body(body_b));
        assert!(!contact.involves_body(BodyId::new(999)));
        assert!(!contact.is_ground_contact());
    }

    #[test]
    fn test_ground_contact() {
        let body = BodyId::new(1);
        let contact =
            ContactInfo::with_ground(body, Point3::new(0.0, 0.0, 0.0), Vector3::z(), 0.01);

        assert!(contact.is_ground_contact());
        assert!(contact.involves_body(body));
    }

    #[test]
    fn test_contact_force() {
        let contact = ContactInfo::new(BodyId::new(1), None, Point3::origin(), Vector3::z(), 0.01)
            .with_impulses(10.0, Vector3::new(1.0, 0.0, 0.0));

        let force = contact.force(0.01);
        assert!((force.z - 1000.0).abs() < 1e-10); // 10 / 0.01 = 1000
        assert!((force.x - 100.0).abs() < 1e-10); // 1 / 0.01 = 100
    }

    #[test]
    fn test_contact_stats() {
        let body = BodyId::new(1);
        let contacts = vec![
            ContactInfo::with_ground(body, Point3::new(1.0, 0.0, 0.0), Vector3::z(), 0.01)
                .with_impulses(0.1, Vector3::zeros()),
            ContactInfo::with_ground(body, Point3::new(-1.0, 0.0, 0.0), Vector3::z(), 0.01)
                .with_impulses(0.1, Vector3::zeros()),
        ];

        let stats = ContactStats::for_body(&contacts, body, 0.01);
        assert_eq!(stats.count, 2);
        assert!(stats.has_contacts());

        // Average position should be at origin
        let avg_pos = stats.average_position.expect("should have position");
        assert!((avg_pos.x).abs() < 1e-10);
    }

    #[test]
    fn test_contact_stats_no_contacts() {
        let stats = ContactStats::for_body(&[], BodyId::new(1), 0.01);
        assert_eq!(stats.count, 0);
        assert!(!stats.has_contacts());
    }

    #[test]
    fn test_pose_observation() {
        let body = BodyId::new(1);
        let pose = Pose::from_position(Point3::new(1.0, 2.0, 3.0));
        let obs = PoseObservation::new(body, pose);

        assert_eq!(obs.body, body);
        assert_eq!(obs.pose.position, pose.position);
    }

    #[test]
    fn test_velocity_observation() {
        let body = BodyId::new(2);
        let twist = Twist::linear(Vector3::new(1.0, 0.0, 0.0));
        let obs = VelocityObservation::new(body, twist);

        assert_eq!(obs.body, body);
        assert_eq!(obs.twist.linear, twist.linear);
    }

    #[test]
    fn test_contact_stats_total_force() {
        let body = BodyId::new(1);
        let contacts = vec![
            ContactInfo::with_ground(body, Point3::origin(), Vector3::z(), 0.01)
                .with_impulses(0.3, Vector3::new(0.4, 0.0, 0.0)),
        ];

        let stats = ContactStats::for_body(&contacts, body, 0.01);
        // Normal: 0.3/0.01 = 30, Tangent: 0.4/0.01 = 40
        // Total: sqrt(30^2 + 40^2) = sqrt(900 + 1600) = sqrt(2500) = 50
        let total = stats.total_force();
        assert!((total - 50.0).abs() < 1e-10);
    }
}
