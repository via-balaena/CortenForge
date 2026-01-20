//! Inertial Measurement Unit (IMU) sensor.
//!
//! An IMU combines an accelerometer and gyroscope to measure:
//! - Linear acceleration (from accelerometer)
//! - Angular velocity (from gyroscope)
//!
//! The sensor can optionally include gravity in the acceleration reading
//! (as real accelerometers do) or exclude it for "proper" acceleration.

use nalgebra::Vector3;
use sim_types::{BodyId, RigidBodyState, SensorObservation, Twist};

use crate::{SensorData, SensorId, SensorReading, SensorType};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for an IMU sensor.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImuConfig {
    /// Position of the sensor in the body's local frame.
    pub local_position: Vector3<f64>,

    /// Orientation of the sensor in the body's local frame.
    /// This allows mounting the sensor at an angle.
    pub local_rotation: nalgebra::UnitQuaternion<f64>,

    /// Whether to include gravity in the acceleration reading.
    /// Real accelerometers measure gravity, so this defaults to true.
    pub include_gravity: bool,

    /// Gravity vector in world frame (default: -9.81 Z).
    pub gravity: Vector3<f64>,

    /// Accelerometer noise standard deviation (m/s^2).
    /// Set to 0 for ideal sensor.
    pub accel_noise_std: f64,

    /// Gyroscope noise standard deviation (rad/s).
    /// Set to 0 for ideal sensor.
    pub gyro_noise_std: f64,

    /// Accelerometer bias (constant offset in sensor frame).
    pub accel_bias: Vector3<f64>,

    /// Gyroscope bias (constant offset in sensor frame).
    pub gyro_bias: Vector3<f64>,
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            local_position: Vector3::zeros(),
            local_rotation: nalgebra::UnitQuaternion::identity(),
            include_gravity: true,
            gravity: Vector3::new(0.0, 0.0, -9.81),
            accel_noise_std: 0.0,
            gyro_noise_std: 0.0,
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
        }
    }
}

impl ImuConfig {
    /// Create a new IMU config with default settings.
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

    /// Set whether to include gravity in acceleration readings.
    #[must_use]
    pub fn with_gravity_included(mut self, include: bool) -> Self {
        self.include_gravity = include;
        self
    }

    /// Set the gravity vector.
    #[must_use]
    pub fn with_gravity(mut self, gravity: Vector3<f64>) -> Self {
        self.gravity = gravity;
        self
    }

    /// Set accelerometer noise.
    #[must_use]
    pub fn with_accel_noise(mut self, std: f64) -> Self {
        self.accel_noise_std = std;
        self
    }

    /// Set gyroscope noise.
    #[must_use]
    pub fn with_gyro_noise(mut self, std: f64) -> Self {
        self.gyro_noise_std = std;
        self
    }

    /// Set accelerometer bias.
    #[must_use]
    pub fn with_accel_bias(mut self, bias: Vector3<f64>) -> Self {
        self.accel_bias = bias;
        self
    }

    /// Set gyroscope bias.
    #[must_use]
    pub fn with_gyro_bias(mut self, bias: Vector3<f64>) -> Self {
        self.gyro_bias = bias;
        self
    }

    /// Create a "perfect" IMU with no noise or bias.
    #[must_use]
    pub fn ideal() -> Self {
        Self {
            include_gravity: false,
            ..Self::default()
        }
    }

    /// Create a realistic IMU with typical consumer-grade noise characteristics.
    #[must_use]
    pub fn realistic() -> Self {
        Self {
            accel_noise_std: 0.01, // ~0.01 m/s^2
            gyro_noise_std: 0.001, // ~0.001 rad/s
            ..Self::default()
        }
    }
}

/// IMU reading containing accelerometer and gyroscope data.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImuReading {
    /// Linear acceleration in sensor frame (m/s^2).
    pub linear_acceleration: Vector3<f64>,
    /// Angular velocity in sensor frame (rad/s).
    pub angular_velocity: Vector3<f64>,
}

impl ImuReading {
    /// Create a new IMU reading.
    #[must_use]
    pub fn new(linear_acceleration: Vector3<f64>, angular_velocity: Vector3<f64>) -> Self {
        Self {
            linear_acceleration,
            angular_velocity,
        }
    }

    /// Create a zero reading.
    #[must_use]
    pub fn zero() -> Self {
        Self {
            linear_acceleration: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
        }
    }

    /// Get the acceleration magnitude.
    #[must_use]
    pub fn acceleration_magnitude(&self) -> f64 {
        self.linear_acceleration.norm()
    }

    /// Get the angular velocity magnitude.
    #[must_use]
    pub fn angular_velocity_magnitude(&self) -> f64 {
        self.angular_velocity.norm()
    }

    /// Convert to `SensorData` for generic handling.
    #[must_use]
    pub fn to_sensor_data(self) -> SensorData {
        SensorData::Imu {
            linear_acceleration: self.linear_acceleration,
            angular_velocity: self.angular_velocity,
        }
    }

    /// Convert to `SensorObservation` for inclusion in simulation observations.
    #[must_use]
    pub fn to_sensor_observation(self, sensor_id: u64, body_id: BodyId) -> SensorObservation {
        SensorObservation::imu(
            sensor_id,
            body_id,
            self.linear_acceleration,
            self.angular_velocity,
        )
    }
}

/// Inertial Measurement Unit sensor.
///
/// Measures linear acceleration and angular velocity of a body.
/// The sensor can be mounted at any position and orientation on the body.
///
/// # Example
///
/// ```
/// use sim_sensor::{Imu, ImuConfig};
/// use sim_types::{BodyId, RigidBodyState, Pose, Twist};
/// use nalgebra::Vector3;
///
/// // Create an IMU attached to body 1
/// let imu = Imu::new(BodyId::new(1), ImuConfig::default());
///
/// // Read from a body state (in practice, you'd get this from a World)
/// let state = RigidBodyState::at_rest(Pose::identity());
/// let reading = imu.read_from_state(&state, 0.0);
///
/// // With gravity included, a stationary body should read ~9.81 m/s^2 upward
/// // (accelerometer measures the reaction to gravity, not the fall)
/// assert!((reading.linear_acceleration.z - 9.81).abs() < 0.01);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Imu {
    /// Unique sensor ID.
    id: SensorId,
    /// Body this sensor is attached to.
    body_id: BodyId,
    /// Optional name for debugging.
    name: Option<String>,
    /// Sensor configuration.
    config: ImuConfig,
    /// Previous velocity for computing acceleration (needed for proper acceleration).
    /// This is optional - if None, we assume zero acceleration from velocity change.
    previous_twist: Option<Twist>,
}

impl Imu {
    /// Create a new IMU sensor.
    #[must_use]
    pub fn new(body_id: BodyId, config: ImuConfig) -> Self {
        Self {
            id: SensorId::new(0), // Will be assigned when added to a manager
            body_id,
            name: None,
            config,
            previous_twist: None,
        }
    }

    /// Create an IMU with a specific ID.
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
    pub fn config(&self) -> &ImuConfig {
        &self.config
    }

    /// Get a mutable reference to the configuration.
    pub fn config_mut(&mut self) -> &mut ImuConfig {
        &mut self.config
    }

    /// Set the sensor ID.
    pub fn set_id(&mut self, id: SensorId) {
        self.id = id;
    }

    /// Read the sensor from body state and acceleration.
    ///
    /// # Arguments
    ///
    /// * `state` - Current state of the body (pose and twist)
    /// * `linear_acceleration` - Linear acceleration of the body in world frame
    ///
    /// # Returns
    ///
    /// IMU reading in the sensor's local frame
    #[must_use]
    pub fn read_with_acceleration(
        &self,
        state: &RigidBodyState,
        linear_acceleration: Vector3<f64>,
    ) -> ImuReading {
        let pose = &state.pose;
        let twist = &state.twist;

        // Transform sensor position to world frame
        let sensor_world_pos = pose.transform_vector(&self.config.local_position);

        // Compute centripetal acceleration at sensor location
        // a_centripetal = omega x (omega x r)
        let omega = &twist.angular;
        let centripetal = omega.cross(&omega.cross(&sensor_world_pos));

        // Compute tangential acceleration at sensor location
        // For now, assume zero angular acceleration (would need derivative of omega)
        // a_tangential = alpha x r
        let tangential = Vector3::zeros();

        // Total acceleration at sensor location in world frame
        let mut accel_world = linear_acceleration + centripetal + tangential;

        // Add gravity if configured (accelerometers measure reaction to gravity)
        // When stationary, accelerometer reads +g (upward) to oppose gravity
        if self.config.include_gravity {
            accel_world -= self.config.gravity;
        }

        // Transform to body frame
        let accel_body = pose.inverse_transform_vector(&accel_world);

        // Transform to sensor frame
        let accel_sensor = self.config.local_rotation.inverse() * accel_body;

        // Angular velocity in body frame
        let omega_body = pose.inverse_transform_vector(omega);

        // Transform to sensor frame
        let omega_sensor = self.config.local_rotation.inverse() * omega_body;

        // Apply bias (no noise in this basic implementation)
        let final_accel = accel_sensor + self.config.accel_bias;
        let final_gyro = omega_sensor + self.config.gyro_bias;

        ImuReading::new(final_accel, final_gyro)
    }

    /// Read the sensor from body state, computing acceleration from velocity change.
    ///
    /// This is a simpler interface when you don't have direct access to acceleration.
    /// It assumes zero linear acceleration from body motion (only gravity and rotation effects).
    #[must_use]
    pub fn read_from_state(&self, state: &RigidBodyState, _dt: f64) -> ImuReading {
        // For now, assume zero linear acceleration from body motion
        // A more complete implementation would track previous state and compute dv/dt
        self.read_with_acceleration(state, Vector3::zeros())
    }

    /// Read the sensor and return a generic [`SensorReading`].
    #[must_use]
    pub fn read_as_sensor_reading(
        &self,
        state: &RigidBodyState,
        linear_acceleration: Vector3<f64>,
        timestamp: f64,
    ) -> SensorReading {
        let reading = self.read_with_acceleration(state, linear_acceleration);
        SensorReading::new(
            self.id,
            SensorType::Imu,
            timestamp,
            reading.to_sensor_data(),
        )
    }

    /// Read the sensor and return a `SensorObservation` for inclusion in simulation observations.
    #[must_use]
    pub fn read_as_observation(
        &self,
        state: &RigidBodyState,
        linear_acceleration: Vector3<f64>,
    ) -> SensorObservation {
        let reading = self.read_with_acceleration(state, linear_acceleration);
        let mut obs = reading.to_sensor_observation(self.id.raw(), self.body_id);
        if let Some(name) = &self.name {
            obs = obs.with_name(name.clone());
        }
        obs
    }

    /// Update the previous twist for acceleration computation.
    pub fn update_previous_twist(&mut self, twist: Twist) {
        self.previous_twist = Some(twist);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::{Point3, UnitQuaternion};
    use sim_types::Pose;

    #[test]
    fn test_imu_config_default() {
        let config = ImuConfig::default();
        assert!(config.include_gravity);
        assert_eq!(config.gravity.z, -9.81);
        assert_eq!(config.accel_noise_std, 0.0);
    }

    #[test]
    fn test_imu_config_builder() {
        let config = ImuConfig::new()
            .with_local_position(Vector3::new(0.1, 0.0, 0.0))
            .with_gravity_included(false)
            .with_accel_noise(0.01);

        assert!(!config.include_gravity);
        assert_eq!(config.local_position.x, 0.1);
        assert_eq!(config.accel_noise_std, 0.01);
    }

    #[test]
    fn test_imu_stationary_with_gravity() {
        let imu = Imu::new(BodyId::new(1), ImuConfig::default());
        let state = RigidBodyState::at_rest(Pose::identity());
        let reading = imu.read_with_acceleration(&state, Vector3::zeros());

        // Stationary body should measure +9.81 m/s^2 in Z (opposing gravity)
        assert_relative_eq!(reading.linear_acceleration.z, 9.81, epsilon = 0.001);
        assert_relative_eq!(reading.angular_velocity.norm(), 0.0, epsilon = 0.001);
    }

    #[test]
    fn test_imu_stationary_without_gravity() {
        let config = ImuConfig::default().with_gravity_included(false);
        let imu = Imu::new(BodyId::new(1), config);
        let state = RigidBodyState::at_rest(Pose::identity());
        let reading = imu.read_with_acceleration(&state, Vector3::zeros());

        // Without gravity, should read zero
        assert_relative_eq!(reading.linear_acceleration.norm(), 0.0, epsilon = 0.001);
    }

    #[test]
    fn test_imu_free_fall() {
        // In free fall, the accelerometer should read zero (no reaction force)
        let config = ImuConfig::default();
        let imu = Imu::new(BodyId::new(1), config);
        let state = RigidBodyState::at_rest(Pose::identity());

        // Apply gravity as the linear acceleration (free fall)
        let gravity = Vector3::new(0.0, 0.0, -9.81);
        let reading = imu.read_with_acceleration(&state, gravity);

        // Acceleration from motion cancels the gravity offset
        assert_relative_eq!(reading.linear_acceleration.norm(), 0.0, epsilon = 0.001);
    }

    #[test]
    fn test_imu_rotating_body() {
        let imu = Imu::new(
            BodyId::new(1),
            ImuConfig::default().with_gravity_included(false),
        );
        let pose = Pose::identity();
        let twist = Twist::angular(Vector3::new(0.0, 0.0, 1.0)); // 1 rad/s around Z
        let state = RigidBodyState::new(pose, twist);

        let reading = imu.read_with_acceleration(&state, Vector3::zeros());

        // Should measure the angular velocity
        assert_relative_eq!(reading.angular_velocity.z, 1.0, epsilon = 0.001);
    }

    #[test]
    fn test_imu_rotated_body() {
        // Body rotated 90 degrees around X axis
        let rotation = UnitQuaternion::from_euler_angles(std::f64::consts::FRAC_PI_2, 0.0, 0.0);
        let pose = Pose::from_position_rotation(Point3::origin(), rotation);
        let state = RigidBodyState::at_rest(pose);

        let imu = Imu::new(BodyId::new(1), ImuConfig::default());
        let reading = imu.read_with_acceleration(&state, Vector3::zeros());

        // Gravity is in -Z world, but body is rotated so sensor Y points up
        // So sensor should read +9.81 in Y
        assert_relative_eq!(reading.linear_acceleration.y, 9.81, epsilon = 0.001);
        assert_relative_eq!(reading.linear_acceleration.z, 0.0, epsilon = 0.001);
    }

    #[test]
    fn test_imu_with_bias() {
        let config = ImuConfig::default()
            .with_gravity_included(false)
            .with_accel_bias(Vector3::new(0.1, 0.0, 0.0))
            .with_gyro_bias(Vector3::new(0.0, 0.01, 0.0));

        let imu = Imu::new(BodyId::new(1), config);
        let state = RigidBodyState::at_rest(Pose::identity());
        let reading = imu.read_with_acceleration(&state, Vector3::zeros());

        // Should measure the bias
        assert_relative_eq!(reading.linear_acceleration.x, 0.1, epsilon = 0.001);
        assert_relative_eq!(reading.angular_velocity.y, 0.01, epsilon = 0.001);
    }

    #[test]
    fn test_imu_sensor_reading() {
        let imu = Imu::new(
            BodyId::new(1),
            ImuConfig::default().with_gravity_included(false),
        )
        .with_id(SensorId::new(42));

        let state = RigidBodyState::at_rest(Pose::identity());
        let reading = imu.read_as_sensor_reading(&state, Vector3::zeros(), 0.001);

        assert_eq!(reading.sensor_id.raw(), 42);
        assert_eq!(reading.sensor_type, SensorType::Imu);
        assert_eq!(reading.timestamp, 0.001);
        assert!(reading.data.is_imu());
    }

    #[test]
    fn test_imu_reading_to_vec() {
        let reading = ImuReading::new(Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0));

        let sensor_data = reading.to_sensor_data();
        let vec = sensor_data.to_vec();
        assert_eq!(vec, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }
}
