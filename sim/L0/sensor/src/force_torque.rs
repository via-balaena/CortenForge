//! Force/Torque sensor.
//!
//! A 6-axis force/torque sensor measures forces and torques acting on a body.
//! These are typically mounted at joints or end-effectors to measure contact forces,
//! reaction forces, or load-cell readings.

use nalgebra::Vector3;
use sim_types::{BodyId, SensorObservation};

use crate::{SensorData, SensorId, SensorReading, SensorType};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for a force/torque sensor.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ForceTorqueSensorConfig {
    /// Position of the sensor in the body's local frame.
    pub local_position: Vector3<f64>,

    /// Orientation of the sensor in the body's local frame.
    pub local_rotation: nalgebra::UnitQuaternion<f64>,

    /// Maximum force magnitude before saturation (N).
    /// Set to infinity for no limit.
    pub max_force: f64,

    /// Maximum torque magnitude before saturation (Nm).
    /// Set to infinity for no limit.
    pub max_torque: f64,

    /// Force noise standard deviation (N).
    pub force_noise_std: f64,

    /// Torque noise standard deviation (Nm).
    pub torque_noise_std: f64,

    /// Force bias (constant offset in sensor frame).
    pub force_bias: Vector3<f64>,

    /// Torque bias (constant offset in sensor frame).
    pub torque_bias: Vector3<f64>,

    /// Resolution/deadband for force readings (N).
    /// Forces below this threshold are reported as zero.
    pub force_deadband: f64,

    /// Resolution/deadband for torque readings (Nm).
    /// Torques below this threshold are reported as zero.
    pub torque_deadband: f64,
}

impl Default for ForceTorqueSensorConfig {
    fn default() -> Self {
        Self {
            local_position: Vector3::zeros(),
            local_rotation: nalgebra::UnitQuaternion::identity(),
            max_force: f64::INFINITY,
            max_torque: f64::INFINITY,
            force_noise_std: 0.0,
            torque_noise_std: 0.0,
            force_bias: Vector3::zeros(),
            torque_bias: Vector3::zeros(),
            force_deadband: 0.0,
            torque_deadband: 0.0,
        }
    }
}

impl ForceTorqueSensorConfig {
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

    /// Set the local rotation of the sensor.
    #[must_use]
    pub fn with_local_rotation(mut self, rotation: nalgebra::UnitQuaternion<f64>) -> Self {
        self.local_rotation = rotation;
        self
    }

    /// Set maximum force before saturation.
    #[must_use]
    pub fn with_max_force(mut self, max: f64) -> Self {
        self.max_force = max;
        self
    }

    /// Set maximum torque before saturation.
    #[must_use]
    pub fn with_max_torque(mut self, max: f64) -> Self {
        self.max_torque = max;
        self
    }

    /// Set force noise.
    #[must_use]
    pub fn with_force_noise(mut self, std: f64) -> Self {
        self.force_noise_std = std;
        self
    }

    /// Set torque noise.
    #[must_use]
    pub fn with_torque_noise(mut self, std: f64) -> Self {
        self.torque_noise_std = std;
        self
    }

    /// Set force bias.
    #[must_use]
    pub fn with_force_bias(mut self, bias: Vector3<f64>) -> Self {
        self.force_bias = bias;
        self
    }

    /// Set torque bias.
    #[must_use]
    pub fn with_torque_bias(mut self, bias: Vector3<f64>) -> Self {
        self.torque_bias = bias;
        self
    }

    /// Set force deadband.
    #[must_use]
    pub fn with_force_deadband(mut self, deadband: f64) -> Self {
        self.force_deadband = deadband;
        self
    }

    /// Set torque deadband.
    #[must_use]
    pub fn with_torque_deadband(mut self, deadband: f64) -> Self {
        self.torque_deadband = deadband;
        self
    }

    /// Create an ideal sensor with no limits, noise, or bias.
    #[must_use]
    pub fn ideal() -> Self {
        Self::default()
    }

    /// Create a realistic sensor with typical industrial-grade characteristics.
    #[must_use]
    pub fn realistic() -> Self {
        Self {
            max_force: 1000.0,      // 1000 N
            max_torque: 100.0,      // 100 Nm
            force_noise_std: 0.1,   // 0.1 N
            torque_noise_std: 0.01, // 0.01 Nm
            force_deadband: 0.5,    // 0.5 N
            torque_deadband: 0.05,  // 0.05 Nm
            ..Self::default()
        }
    }
}

/// Force/torque sensor reading.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ForceTorqueReading {
    /// Force in sensor frame (N).
    pub force: Vector3<f64>,
    /// Torque in sensor frame (Nm).
    pub torque: Vector3<f64>,
}

impl ForceTorqueReading {
    /// Create a new force/torque reading.
    #[must_use]
    pub fn new(force: Vector3<f64>, torque: Vector3<f64>) -> Self {
        Self { force, torque }
    }

    /// Create a zero reading.
    #[must_use]
    pub fn zero() -> Self {
        Self {
            force: Vector3::zeros(),
            torque: Vector3::zeros(),
        }
    }

    /// Get the force magnitude.
    #[must_use]
    pub fn force_magnitude(&self) -> f64 {
        self.force.norm()
    }

    /// Get the torque magnitude.
    #[must_use]
    pub fn torque_magnitude(&self) -> f64 {
        self.torque.norm()
    }

    /// Convert to `SensorData` for generic handling.
    #[must_use]
    pub fn to_sensor_data(self) -> SensorData {
        SensorData::ForceTorque {
            force: self.force,
            torque: self.torque,
        }
    }

    /// Combine with another reading (add forces and torques).
    #[must_use]
    pub fn add(&self, other: &Self) -> Self {
        Self {
            force: self.force + other.force,
            torque: self.torque + other.torque,
        }
    }

    /// Negate the reading.
    #[must_use]
    pub fn negate(&self) -> Self {
        Self {
            force: -self.force,
            torque: -self.torque,
        }
    }

    /// Convert to `SensorObservation` for inclusion in simulation observations.
    #[must_use]
    pub fn to_sensor_observation(self, sensor_id: u64, body_id: BodyId) -> SensorObservation {
        SensorObservation::force_torque(sensor_id, body_id, self.force, self.torque)
    }
}

impl Default for ForceTorqueReading {
    fn default() -> Self {
        Self::zero()
    }
}

/// 6-axis force/torque sensor.
///
/// Measures forces and torques acting on a body. Can be used to measure:
/// - Contact forces (from external world)
/// - Joint reaction forces
/// - End-effector loads
///
/// # Example
///
/// ```
/// use sim_sensor::{ForceTorqueSensor, ForceTorqueSensorConfig, ForceTorqueReading};
/// use sim_types::BodyId;
/// use nalgebra::Vector3;
///
/// // Create a sensor attached to body 1
/// let sensor = ForceTorqueSensor::new(BodyId::new(1), ForceTorqueSensorConfig::default());
///
/// // Process a force/torque input
/// let force = Vector3::new(10.0, 0.0, 0.0);
/// let torque = Vector3::new(0.0, 0.0, 1.0);
/// let reading = sensor.process(force, torque);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ForceTorqueSensor {
    /// Unique sensor ID.
    id: SensorId,
    /// Body this sensor is attached to.
    body_id: BodyId,
    /// Optional name for debugging.
    name: Option<String>,
    /// Sensor configuration.
    config: ForceTorqueSensorConfig,
}

impl ForceTorqueSensor {
    /// Create a new force/torque sensor.
    #[must_use]
    pub fn new(body_id: BodyId, config: ForceTorqueSensorConfig) -> Self {
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
    pub fn config(&self) -> &ForceTorqueSensorConfig {
        &self.config
    }

    /// Get a mutable reference to the configuration.
    pub fn config_mut(&mut self) -> &mut ForceTorqueSensorConfig {
        &mut self.config
    }

    /// Set the sensor ID.
    pub fn set_id(&mut self, id: SensorId) {
        self.id = id;
    }

    /// Process a force/torque input through the sensor model.
    ///
    /// Applies:
    /// 1. Coordinate transform to sensor frame
    /// 2. Deadband filtering
    /// 3. Saturation limits
    /// 4. Bias offset
    ///
    /// # Arguments
    ///
    /// * `force_body` - Force in body frame (N)
    /// * `torque_body` - Torque in body frame (Nm)
    #[must_use]
    pub fn process(
        &self,
        force_body: Vector3<f64>,
        torque_body: Vector3<f64>,
    ) -> ForceTorqueReading {
        // Transform to sensor frame
        let force_sensor = self.config.local_rotation.inverse() * force_body;
        let torque_sensor = self.config.local_rotation.inverse() * torque_body;

        // Apply deadband
        let force_filtered = if force_sensor.norm() < self.config.force_deadband {
            Vector3::zeros()
        } else {
            force_sensor
        };

        let torque_filtered = if torque_sensor.norm() < self.config.torque_deadband {
            Vector3::zeros()
        } else {
            torque_sensor
        };

        // Apply saturation
        let force_saturated = Self::saturate_vector(force_filtered, self.config.max_force);
        let torque_saturated = Self::saturate_vector(torque_filtered, self.config.max_torque);

        // Apply bias
        let force_final = force_saturated + self.config.force_bias;
        let torque_final = torque_saturated + self.config.torque_bias;

        ForceTorqueReading::new(force_final, torque_final)
    }

    /// Process and return a generic [`SensorReading`].
    #[must_use]
    pub fn process_as_sensor_reading(
        &self,
        force_body: Vector3<f64>,
        torque_body: Vector3<f64>,
        timestamp: f64,
    ) -> SensorReading {
        let reading = self.process(force_body, torque_body);
        SensorReading::new(
            self.id,
            SensorType::ForceTorque,
            timestamp,
            reading.to_sensor_data(),
        )
    }

    /// Process and return a `SensorObservation` for inclusion in simulation observations.
    #[must_use]
    pub fn process_as_observation(
        &self,
        force_body: Vector3<f64>,
        torque_body: Vector3<f64>,
    ) -> SensorObservation {
        let reading = self.process(force_body, torque_body);
        let mut obs = reading.to_sensor_observation(self.id.raw(), self.body_id);
        if let Some(name) = &self.name {
            obs = obs.with_name(name.clone());
        }
        obs
    }

    /// Saturate a vector to a maximum magnitude.
    fn saturate_vector(v: Vector3<f64>, max: f64) -> Vector3<f64> {
        let magnitude = v.norm();
        if magnitude > max && magnitude > 0.0 {
            v * (max / magnitude)
        } else {
            v
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::UnitQuaternion;

    #[test]
    fn test_config_default() {
        let config = ForceTorqueSensorConfig::default();
        assert!(config.max_force.is_infinite());
        assert!(config.max_torque.is_infinite());
        assert_eq!(config.force_noise_std, 0.0);
    }

    #[test]
    fn test_config_builder() {
        let config = ForceTorqueSensorConfig::new()
            .with_max_force(100.0)
            .with_max_torque(10.0)
            .with_force_deadband(0.5);

        assert_eq!(config.max_force, 100.0);
        assert_eq!(config.max_torque, 10.0);
        assert_eq!(config.force_deadband, 0.5);
    }

    #[test]
    fn test_reading_zero() {
        let reading = ForceTorqueReading::zero();
        assert_eq!(reading.force_magnitude(), 0.0);
        assert_eq!(reading.torque_magnitude(), 0.0);
    }

    #[test]
    fn test_reading_magnitude() {
        let reading =
            ForceTorqueReading::new(Vector3::new(3.0, 4.0, 0.0), Vector3::new(0.0, 0.0, 5.0));
        assert_relative_eq!(reading.force_magnitude(), 5.0, epsilon = 0.001);
        assert_relative_eq!(reading.torque_magnitude(), 5.0, epsilon = 0.001);
    }

    #[test]
    fn test_reading_add() {
        let r1 = ForceTorqueReading::new(Vector3::new(1.0, 0.0, 0.0), Vector3::zeros());
        let r2 = ForceTorqueReading::new(Vector3::new(2.0, 0.0, 0.0), Vector3::zeros());
        let combined = r1.add(&r2);
        assert_relative_eq!(combined.force.x, 3.0, epsilon = 0.001);
    }

    #[test]
    fn test_sensor_passthrough() {
        let sensor = ForceTorqueSensor::new(BodyId::new(1), ForceTorqueSensorConfig::ideal());
        let force = Vector3::new(10.0, 20.0, 30.0);
        let torque = Vector3::new(1.0, 2.0, 3.0);
        let reading = sensor.process(force, torque);

        assert_relative_eq!(reading.force.x, 10.0, epsilon = 0.001);
        assert_relative_eq!(reading.force.y, 20.0, epsilon = 0.001);
        assert_relative_eq!(reading.force.z, 30.0, epsilon = 0.001);
        assert_relative_eq!(reading.torque.x, 1.0, epsilon = 0.001);
    }

    #[test]
    fn test_sensor_saturation() {
        let config = ForceTorqueSensorConfig::new()
            .with_max_force(10.0)
            .with_max_torque(1.0);
        let sensor = ForceTorqueSensor::new(BodyId::new(1), config);

        // Force that exceeds limit
        let force = Vector3::new(100.0, 0.0, 0.0);
        let torque = Vector3::new(10.0, 0.0, 0.0);
        let reading = sensor.process(force, torque);

        assert_relative_eq!(reading.force.x, 10.0, epsilon = 0.001);
        assert_relative_eq!(reading.torque.x, 1.0, epsilon = 0.001);
    }

    #[test]
    fn test_sensor_deadband() {
        let config = ForceTorqueSensorConfig::new()
            .with_force_deadband(1.0)
            .with_torque_deadband(0.1);
        let sensor = ForceTorqueSensor::new(BodyId::new(1), config);

        // Forces below deadband
        let force = Vector3::new(0.5, 0.0, 0.0);
        let torque = Vector3::new(0.05, 0.0, 0.0);
        let reading = sensor.process(force, torque);

        assert_eq!(reading.force.norm(), 0.0);
        assert_eq!(reading.torque.norm(), 0.0);

        // Forces above deadband
        let force = Vector3::new(2.0, 0.0, 0.0);
        let torque = Vector3::new(0.2, 0.0, 0.0);
        let reading = sensor.process(force, torque);

        assert!(reading.force.norm() > 0.0);
        assert!(reading.torque.norm() > 0.0);
    }

    #[test]
    fn test_sensor_bias() {
        let config = ForceTorqueSensorConfig::new()
            .with_force_bias(Vector3::new(1.0, 0.0, 0.0))
            .with_torque_bias(Vector3::new(0.0, 0.1, 0.0));
        let sensor = ForceTorqueSensor::new(BodyId::new(1), config);

        let reading = sensor.process(Vector3::zeros(), Vector3::zeros());

        assert_relative_eq!(reading.force.x, 1.0, epsilon = 0.001);
        assert_relative_eq!(reading.torque.y, 0.1, epsilon = 0.001);
    }

    #[test]
    fn test_sensor_rotation() {
        // Sensor rotated 90 degrees around Z axis
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let config = ForceTorqueSensorConfig::new().with_local_rotation(rotation);
        let sensor = ForceTorqueSensor::new(BodyId::new(1), config);

        // Force in +X body frame
        let force = Vector3::new(10.0, 0.0, 0.0);
        let reading = sensor.process(force, Vector3::zeros());

        // After inverse rotation, should be in +Y sensor frame
        assert_relative_eq!(reading.force.x, 0.0, epsilon = 0.001);
        assert_relative_eq!(reading.force.y, -10.0, epsilon = 0.001);
    }

    #[test]
    fn test_sensor_reading_output() {
        let sensor = ForceTorqueSensor::new(BodyId::new(1), ForceTorqueSensorConfig::ideal())
            .with_id(SensorId::new(42));

        let reading = sensor.process_as_sensor_reading(
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(4.0, 5.0, 6.0),
            0.001,
        );

        assert_eq!(reading.sensor_id.raw(), 42);
        assert_eq!(reading.sensor_type, SensorType::ForceTorque);
        assert!(reading.data.is_force_torque());
    }
}
