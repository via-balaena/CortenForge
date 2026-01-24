//! Touch/contact sensor.
//!
//! A touch sensor detects contact with other bodies and measures contact forces.
//! These can be simple binary sensors (contact/no contact) or measure force magnitude.

use nalgebra::Vector3;
use sim_types::{BodyId, ContactInfo, SensorObservation};

use crate::{SensorData, SensorId, SensorReading, SensorType};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for a touch sensor.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TouchSensorConfig {
    /// Position of the sensor in the body's local frame.
    /// Contact points within `detection_radius` of this position are detected.
    pub local_position: Vector3<f64>,

    /// Radius around the sensor position to detect contacts (meters).
    /// Set to infinity to detect all contacts on the body.
    pub detection_radius: f64,

    /// Minimum force threshold to register a contact (N).
    /// Forces below this are ignored.
    pub force_threshold: f64,

    /// Whether to track individual contact points or just aggregate.
    pub track_individual_contacts: bool,

    /// Maximum number of contact points to track (if tracking individuals).
    pub max_contacts: usize,

    /// Whether to filter by specific body IDs (empty = all bodies).
    /// Only contacts with bodies in this list are detected.
    pub filter_bodies: Vec<BodyId>,

    /// Whether to include ground/environment contacts.
    pub include_ground_contacts: bool,
}

impl Default for TouchSensorConfig {
    fn default() -> Self {
        Self {
            local_position: Vector3::zeros(),
            detection_radius: f64::INFINITY,
            force_threshold: 0.0,
            track_individual_contacts: false,
            max_contacts: 10,
            filter_bodies: Vec::new(),
            include_ground_contacts: true,
        }
    }
}

impl TouchSensorConfig {
    /// Create a new config with default settings.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the local position of the sensor.
    #[must_use]
    pub fn with_local_position(mut self, position: Vector3<f64>) -> Self {
        self.local_position = position;
        self
    }

    /// Set the detection radius.
    #[must_use]
    pub fn with_detection_radius(mut self, radius: f64) -> Self {
        self.detection_radius = radius;
        self
    }

    /// Set the force threshold.
    #[must_use]
    pub fn with_force_threshold(mut self, threshold: f64) -> Self {
        self.force_threshold = threshold;
        self
    }

    /// Enable tracking of individual contact points.
    #[must_use]
    pub fn with_individual_tracking(mut self, track: bool) -> Self {
        self.track_individual_contacts = track;
        self
    }

    /// Set the maximum number of contacts to track.
    #[must_use]
    pub fn with_max_contacts(mut self, max: usize) -> Self {
        self.max_contacts = max;
        self
    }

    /// Filter contacts to specific bodies.
    #[must_use]
    pub fn with_body_filter(mut self, bodies: Vec<BodyId>) -> Self {
        self.filter_bodies = bodies;
        self
    }

    /// Set whether to include ground contacts.
    #[must_use]
    pub fn with_ground_contacts(mut self, include: bool) -> Self {
        self.include_ground_contacts = include;
        self
    }

    /// Create a binary touch sensor (just detects contact, no force).
    #[must_use]
    pub fn binary() -> Self {
        Self::default()
    }

    /// Create a force-sensitive touch sensor.
    #[must_use]
    pub fn force_sensitive(threshold: f64) -> Self {
        Self {
            force_threshold: threshold,
            ..Self::default()
        }
    }

    /// Create a localized touch sensor at a specific position.
    #[must_use]
    pub fn localized(position: Vector3<f64>, radius: f64) -> Self {
        Self {
            local_position: position,
            detection_radius: radius,
            ..Self::default()
        }
    }
}

/// Touch sensor reading.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TouchReading {
    /// Whether any contact is detected.
    pub in_contact: bool,
    /// Number of contact points detected.
    pub contact_count: usize,
    /// Total contact force magnitude (N).
    pub total_force: f64,
    /// Average contact normal (if in contact).
    pub average_normal: Option<Vector3<f64>>,
    /// Individual contact forces (if tracking).
    pub contact_forces: Vec<f64>,
    /// Individual contact normals (if tracking).
    pub contact_normals: Vec<Vector3<f64>>,
}

impl TouchReading {
    /// Create a reading with no contact.
    #[must_use]
    pub fn no_contact() -> Self {
        Self {
            in_contact: false,
            contact_count: 0,
            total_force: 0.0,
            average_normal: None,
            contact_forces: Vec::new(),
            contact_normals: Vec::new(),
        }
    }

    /// Create a reading with contact.
    #[must_use]
    pub fn with_contact(
        contact_count: usize,
        total_force: f64,
        average_normal: Vector3<f64>,
    ) -> Self {
        Self {
            in_contact: true,
            contact_count,
            total_force,
            average_normal: Some(average_normal),
            contact_forces: Vec::new(),
            contact_normals: Vec::new(),
        }
    }

    /// Add individual contact data.
    pub fn add_contact(&mut self, force: f64, normal: Vector3<f64>) {
        self.contact_forces.push(force);
        self.contact_normals.push(normal);
    }

    /// Check if there is contact.
    #[must_use]
    pub fn is_in_contact(&self) -> bool {
        self.in_contact
    }

    /// Get the maximum contact force.
    #[must_use]
    pub fn max_force(&self) -> f64 {
        self.contact_forces.iter().copied().fold(0.0, f64::max).max(
            if self.in_contact && self.contact_forces.is_empty() {
                self.total_force
            } else {
                0.0
            },
        )
    }

    /// Convert to `SensorData` for generic handling.
    #[must_use]
    pub fn to_sensor_data(&self) -> SensorData {
        SensorData::Touch {
            in_contact: self.in_contact,
            contact_count: self.contact_count,
            contact_force: self.total_force,
            contact_normal: self.average_normal,
        }
    }

    /// Convert to `SensorObservation` for inclusion in simulation observations.
    #[must_use]
    pub fn to_sensor_observation(&self, sensor_id: u64, body_id: BodyId) -> SensorObservation {
        SensorObservation::touch(
            sensor_id,
            body_id,
            self.in_contact,
            self.contact_count,
            self.total_force,
            self.average_normal,
        )
    }
}

impl Default for TouchReading {
    fn default() -> Self {
        Self::no_contact()
    }
}

/// Touch/contact sensor.
///
/// Detects contact with other bodies and measures contact forces.
///
/// # Example
///
/// ```
/// use sim_sensor::{TouchSensor, TouchSensorConfig};
/// use sim_types::{BodyId, ContactInfo};
/// use nalgebra::{Point3, Vector3};
///
/// // Create a touch sensor for body 1
/// let sensor = TouchSensor::new(BodyId::new(1), TouchSensorConfig::default());
///
/// // Process contacts (would normally come from simulation)
/// let contacts = vec![
///     ContactInfo::with_ground(BodyId::new(1), Point3::origin(), Vector3::z(), 0.01),
/// ];
/// let reading = sensor.read(&contacts, 0.001);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TouchSensor {
    /// Unique sensor ID.
    id: SensorId,
    /// Body this sensor is attached to.
    body_id: BodyId,
    /// Optional name for debugging.
    name: Option<String>,
    /// Sensor configuration.
    config: TouchSensorConfig,
}

impl TouchSensor {
    /// Create a new touch sensor.
    #[must_use]
    pub fn new(body_id: BodyId, config: TouchSensorConfig) -> Self {
        Self {
            id: SensorId::new(0),
            body_id,
            name: None,
            config,
        }
    }

    /// Create a sensor with a specific ID.
    #[must_use]
    pub fn with_id(mut self, id: SensorId) -> Self {
        self.id = id;
        self
    }

    /// Set the sensor name.
    #[must_use]
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Get the sensor ID.
    #[must_use]
    pub fn id(&self) -> SensorId {
        self.id
    }

    /// Get the body ID this sensor is attached to.
    #[must_use]
    pub fn body_id(&self) -> BodyId {
        self.body_id
    }

    /// Get the sensor name.
    #[must_use]
    pub fn name(&self) -> Option<&str> {
        self.name.as_deref()
    }

    /// Get the sensor configuration.
    #[must_use]
    pub fn config(&self) -> &TouchSensorConfig {
        &self.config
    }

    /// Get a mutable reference to the configuration.
    pub fn config_mut(&mut self) -> &mut TouchSensorConfig {
        &mut self.config
    }

    /// Set the sensor ID.
    pub fn set_id(&mut self, id: SensorId) {
        self.id = id;
    }

    /// Read the sensor from a list of contacts.
    ///
    /// # Arguments
    ///
    /// * `contacts` - All contacts in the simulation
    /// * `dt` - Time step for force computation
    #[must_use]
    pub fn read(&self, contacts: &[ContactInfo], dt: f64) -> TouchReading {
        // Filter contacts involving this body
        let relevant_contacts: Vec<_> = contacts
            .iter()
            .filter(|c| self.is_relevant_contact(c))
            .collect();

        if relevant_contacts.is_empty() {
            return TouchReading::no_contact();
        }

        // Compute aggregate values
        let mut total_force = 0.0;
        let mut sum_normal = Vector3::zeros();
        let mut contact_forces = Vec::new();
        let mut contact_normals = Vec::new();
        let mut threshold_passed_count = 0;

        for contact in &relevant_contacts {
            let force_vec = contact.force(dt);
            let force_mag = force_vec.norm();

            // Skip contacts below force threshold
            if force_mag < self.config.force_threshold {
                continue;
            }

            threshold_passed_count += 1;
            total_force += force_mag;
            sum_normal += contact.normal;

            if self.config.track_individual_contacts
                && contact_forces.len() < self.config.max_contacts
            {
                contact_forces.push(force_mag);
                contact_normals.push(contact.normal);
            }
        }

        let contact_count = if self.config.track_individual_contacts {
            contact_forces.len()
        } else {
            threshold_passed_count
        };

        if contact_count == 0 {
            return TouchReading::no_contact();
        }

        let average_normal = if sum_normal.norm() > 1e-10 {
            sum_normal.normalize()
        } else {
            Vector3::z() // Default to up if no clear normal
        };

        let mut reading = TouchReading::with_contact(contact_count, total_force, average_normal);
        reading.contact_forces = contact_forces;
        reading.contact_normals = contact_normals;

        reading
    }

    /// Read and return a generic [`SensorReading`].
    #[must_use]
    pub fn read_as_sensor_reading(
        &self,
        contacts: &[ContactInfo],
        dt: f64,
        timestamp: f64,
    ) -> SensorReading {
        let reading = self.read(contacts, dt);
        SensorReading::new(
            self.id,
            SensorType::Touch,
            timestamp,
            reading.to_sensor_data(),
        )
    }

    /// Read and return a `SensorObservation` for inclusion in simulation observations.
    #[must_use]
    pub fn read_as_observation(&self, contacts: &[ContactInfo], dt: f64) -> SensorObservation {
        let reading = self.read(contacts, dt);
        let mut obs = reading.to_sensor_observation(self.id.raw(), self.body_id);
        if let Some(name) = &self.name {
            obs = obs.with_name(name.clone());
        }
        obs
    }

    /// Check if a contact is relevant to this sensor.
    fn is_relevant_contact(&self, contact: &ContactInfo) -> bool {
        // Must involve our body
        if !contact.involves_body(self.body_id) {
            return false;
        }

        // Filter ground contacts if configured
        if contact.is_ground_contact() && !self.config.include_ground_contacts {
            return false;
        }

        // Filter by body list if configured
        if !self.config.filter_bodies.is_empty() {
            let other_body = if contact.body_a == self.body_id {
                contact.body_b
            } else {
                Some(contact.body_a)
            };

            if let Some(other) = other_body {
                if !self.config.filter_bodies.contains(&other) {
                    return false;
                }
            } else {
                // Ground contact, already handled above
            }
        }

        // Check detection radius (would need body pose for proper check)
        // For now, detection_radius = infinity means all contacts pass
        // A proper implementation would transform contact.position to body frame
        // and check distance to local_position
        if self.config.detection_radius.is_finite() {
            // TODO: Implement proper distance check when we have body pose
            // For now, accept all contacts when radius is set
        }

        true
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::option_if_let_else,
    clippy::panic
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::Point3;

    fn make_contact(body_a: BodyId, body_b: Option<BodyId>, impulse: f64) -> ContactInfo {
        let mut contact = if let Some(b) = body_b {
            ContactInfo::between_bodies(body_a, b, Point3::origin(), Vector3::z(), 0.01)
        } else {
            ContactInfo::with_ground(body_a, Point3::origin(), Vector3::z(), 0.01)
        };
        contact = contact.with_impulses(impulse, Vector3::zeros());
        contact
    }

    #[test]
    fn test_config_default() {
        let config = TouchSensorConfig::default();
        assert!(config.detection_radius.is_infinite());
        assert_eq!(config.force_threshold, 0.0);
        assert!(config.include_ground_contacts);
    }

    #[test]
    fn test_config_builder() {
        let config = TouchSensorConfig::new()
            .with_detection_radius(0.1)
            .with_force_threshold(1.0)
            .with_ground_contacts(false);

        assert_eq!(config.detection_radius, 0.1);
        assert_eq!(config.force_threshold, 1.0);
        assert!(!config.include_ground_contacts);
    }

    #[test]
    fn test_reading_no_contact() {
        let reading = TouchReading::no_contact();
        assert!(!reading.is_in_contact());
        assert_eq!(reading.contact_count, 0);
        assert_eq!(reading.total_force, 0.0);
    }

    #[test]
    fn test_reading_with_contact() {
        let reading = TouchReading::with_contact(3, 10.5, Vector3::z());
        assert!(reading.is_in_contact());
        assert_eq!(reading.contact_count, 3);
        assert_eq!(reading.total_force, 10.5);
    }

    #[test]
    fn test_sensor_no_contacts() {
        let sensor = TouchSensor::new(BodyId::new(1), TouchSensorConfig::default());
        let reading = sensor.read(&[], 0.001);
        assert!(!reading.is_in_contact());
    }

    #[test]
    fn test_sensor_with_ground_contact() {
        let sensor = TouchSensor::new(BodyId::new(1), TouchSensorConfig::default());
        let contacts = vec![make_contact(BodyId::new(1), None, 0.01)];
        let reading = sensor.read(&contacts, 0.001);

        assert!(reading.is_in_contact());
        assert_eq!(reading.contact_count, 1);
    }

    #[test]
    fn test_sensor_filters_other_bodies() {
        let sensor = TouchSensor::new(BodyId::new(1), TouchSensorConfig::default());
        // Contact between bodies 2 and 3 (not involving body 1)
        let contacts = vec![make_contact(BodyId::new(2), Some(BodyId::new(3)), 0.01)];
        let reading = sensor.read(&contacts, 0.001);

        assert!(!reading.is_in_contact());
    }

    #[test]
    fn test_sensor_excludes_ground_contacts() {
        let config = TouchSensorConfig::new().with_ground_contacts(false);
        let sensor = TouchSensor::new(BodyId::new(1), config);
        let contacts = vec![make_contact(BodyId::new(1), None, 0.01)];
        let reading = sensor.read(&contacts, 0.001);

        assert!(!reading.is_in_contact());
    }

    #[test]
    fn test_sensor_body_filter() {
        let config = TouchSensorConfig::new().with_body_filter(vec![BodyId::new(2)]);
        let sensor = TouchSensor::new(BodyId::new(1), config);

        // Contact with body 2 (in filter list)
        let contacts = vec![make_contact(BodyId::new(1), Some(BodyId::new(2)), 0.01)];
        let reading = sensor.read(&contacts, 0.001);
        assert!(reading.is_in_contact());

        // Contact with body 3 (not in filter list)
        let contacts = vec![make_contact(BodyId::new(1), Some(BodyId::new(3)), 0.01)];
        let reading = sensor.read(&contacts, 0.001);
        assert!(!reading.is_in_contact());
    }

    #[test]
    fn test_sensor_force_threshold() {
        // Threshold of 5 N
        let config = TouchSensorConfig::new().with_force_threshold(5.0);
        let sensor = TouchSensor::new(BodyId::new(1), config);

        // Impulse of 0.001 with dt=0.001 gives force of 1 N (below threshold)
        let contacts = vec![make_contact(BodyId::new(1), None, 0.001)];
        let reading = sensor.read(&contacts, 0.001);
        assert!(!reading.is_in_contact()); // Below threshold

        // Impulse of 0.01 with dt=0.001 gives force of 10 N (above threshold)
        let contacts = vec![make_contact(BodyId::new(1), None, 0.01)];
        let reading = sensor.read(&contacts, 0.001);
        assert!(reading.is_in_contact()); // Above threshold
    }

    #[test]
    fn test_sensor_multiple_contacts() {
        let sensor = TouchSensor::new(BodyId::new(1), TouchSensorConfig::default());
        let contacts = vec![
            make_contact(BodyId::new(1), None, 0.01),
            make_contact(BodyId::new(1), Some(BodyId::new(2)), 0.02),
        ];
        let reading = sensor.read(&contacts, 0.001);

        assert!(reading.is_in_contact());
        assert_eq!(reading.contact_count, 2);
        // Total force = 10 + 20 = 30 N
        assert_relative_eq!(reading.total_force, 30.0, epsilon = 0.001);
    }

    #[test]
    fn test_sensor_individual_tracking() {
        let config = TouchSensorConfig::new().with_individual_tracking(true);
        let sensor = TouchSensor::new(BodyId::new(1), config);

        let contacts = vec![
            make_contact(BodyId::new(1), None, 0.01),
            make_contact(BodyId::new(1), None, 0.02),
        ];
        let reading = sensor.read(&contacts, 0.001);

        assert_eq!(reading.contact_forces.len(), 2);
        assert_eq!(reading.contact_normals.len(), 2);
    }

    #[test]
    fn test_sensor_reading_output() {
        let sensor = TouchSensor::new(BodyId::new(1), TouchSensorConfig::default())
            .with_id(SensorId::new(42));

        let contacts = vec![make_contact(BodyId::new(1), None, 0.01)];
        let reading = sensor.read_as_sensor_reading(&contacts, 0.001, 0.1);

        assert_eq!(reading.sensor_id.raw(), 42);
        assert_eq!(reading.sensor_type, SensorType::Touch);
        assert_eq!(reading.timestamp, 0.1);
        assert!(reading.data.is_touch());
    }

    #[test]
    fn test_reading_to_sensor_data() {
        let reading = TouchReading::with_contact(2, 15.0, Vector3::z());
        let data = reading.to_sensor_data();

        match data {
            SensorData::Touch {
                in_contact,
                contact_count,
                contact_force,
                contact_normal,
            } => {
                assert!(in_contact);
                assert_eq!(contact_count, 2);
                assert_eq!(contact_force, 15.0);
                assert!(contact_normal.is_some());
            }
            _ => panic!("Expected Touch data"),
        }
    }
}
