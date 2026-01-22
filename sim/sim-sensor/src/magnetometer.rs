//! Magnetometer sensor for magnetic field measurement.
//!
//! A magnetometer measures the magnetic field at its location, typically used
//! for compass-like heading estimation. In most robotics applications, this
//! measures Earth's magnetic field to determine heading (yaw).
//!
//! The sensor can be configured with:
//! - Local mounting position and orientation
//! - Reference magnetic field (Earth's field varies by location)
//! - Noise and bias characteristics
//! - Hard-iron and soft-iron distortion modeling

use nalgebra::{Matrix3, Vector3};
use sim_types::{BodyId, Pose, SensorObservation};

use crate::{SensorData, SensorId, SensorReading, SensorType};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for a magnetometer sensor.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MagnetometerConfig {
    /// Position of the sensor in the body's local frame.
    pub local_position: Vector3<f64>,

    /// Orientation of the sensor in the body's local frame.
    pub local_rotation: nalgebra::UnitQuaternion<f64>,

    /// Earth's magnetic field vector in world frame (Tesla).
    /// This varies by geographic location. Default is approximately correct
    /// for mid-latitudes in the Northern Hemisphere.
    ///
    /// The field points roughly North and down into the Earth.
    /// Typical magnitude: ~25-65 μT (microtesla).
    pub earth_field: Vector3<f64>,

    /// Noise standard deviation (Tesla).
    /// Set to 0 for an ideal sensor.
    pub noise_std: f64,

    /// Constant bias in sensor frame (Tesla).
    /// Represents hard-iron distortion from nearby magnetic materials.
    pub hard_iron_bias: Vector3<f64>,

    /// Soft-iron distortion matrix.
    /// Represents distortion from nearby ferromagnetic materials.
    /// Identity matrix means no soft-iron distortion.
    pub soft_iron_matrix: Matrix3<f64>,

    /// Temperature coefficient (change per degree C).
    /// Most magnetometers have some temperature drift.
    pub temperature_coefficient: f64,
}

impl Default for MagnetometerConfig {
    fn default() -> Self {
        // Earth's field varies by location. This is roughly correct for:
        // - San Francisco area (~50 μT total, pointing North and down)
        // - Declination ~13° East (ignored for simplicity)
        // - Inclination ~61° down from horizontal
        //
        // In NED (North-East-Down) frame:
        // - North component: ~23 μT
        // - Down component: ~42 μT
        //
        // Converting to our Z-up frame (X=East, Y=North, Z=Up):
        let north_component = 23e-6; // Tesla
        let down_component = 42e-6; // Tesla

        Self {
            local_position: Vector3::zeros(),
            local_rotation: nalgebra::UnitQuaternion::identity(),
            earth_field: Vector3::new(0.0, north_component, -down_component),
            noise_std: 0.0,
            hard_iron_bias: Vector3::zeros(),
            soft_iron_matrix: Matrix3::identity(),
            temperature_coefficient: 0.0,
        }
    }
}

impl MagnetometerConfig {
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

    /// Set the Earth's magnetic field vector in world frame.
    #[must_use]
    pub fn with_earth_field(mut self, field: Vector3<f64>) -> Self {
        self.earth_field = field;
        self
    }

    /// Set the noise standard deviation.
    #[must_use]
    pub fn with_noise(mut self, std: f64) -> Self {
        self.noise_std = std;
        self
    }

    /// Set the hard-iron bias.
    #[must_use]
    pub fn with_hard_iron_bias(mut self, bias: Vector3<f64>) -> Self {
        self.hard_iron_bias = bias;
        self
    }

    /// Set the soft-iron distortion matrix.
    #[must_use]
    pub fn with_soft_iron_matrix(mut self, matrix: Matrix3<f64>) -> Self {
        self.soft_iron_matrix = matrix;
        self
    }

    /// Create an ideal (perfect) magnetometer.
    #[must_use]
    pub fn ideal() -> Self {
        Self::default()
    }

    /// Create a realistic consumer-grade magnetometer with typical noise.
    #[must_use]
    pub fn realistic() -> Self {
        Self {
            noise_std: 0.5e-6, // ~0.5 μT typical for consumer magnetometers
            ..Self::default()
        }
    }

    /// Create a magnetometer with calibration errors (hard/soft iron).
    #[must_use]
    pub fn with_calibration_errors(hard_iron: Vector3<f64>, soft_iron_scale: Vector3<f64>) -> Self {
        Self {
            hard_iron_bias: hard_iron,
            soft_iron_matrix: Matrix3::from_diagonal(&soft_iron_scale),
            ..Self::default()
        }
    }

    /// Set Earth's field for a specific geographic location.
    ///
    /// # Arguments
    ///
    /// * `declination` - Angle east of true north (radians)
    /// * `inclination` - Angle below horizontal (radians, positive = down)
    /// * `intensity` - Total field strength (Tesla)
    #[must_use]
    pub fn for_location(declination: f64, inclination: f64, intensity: f64) -> Self {
        // Horizontal component
        let horizontal = intensity * inclination.cos();
        // Vertical component (positive inclination = field pointing down)
        let vertical = intensity * inclination.sin();

        // In our Y=North, X=East, Z=Up frame:
        // North = Y * cos(decl) - X * sin(decl) -- but we simplify assuming decl=0 for now
        // For proper declination handling:
        let north = horizontal * declination.cos();
        let east = horizontal * declination.sin();
        let down = vertical;

        Self {
            earth_field: Vector3::new(east, north, -down),
            ..Self::default()
        }
    }
}

/// Magnetometer reading containing the measured magnetic field.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MagnetometerReading {
    /// Magnetic field vector in sensor frame (Tesla).
    pub magnetic_field: Vector3<f64>,
}

impl MagnetometerReading {
    /// Create a new magnetometer reading.
    #[must_use]
    pub fn new(magnetic_field: Vector3<f64>) -> Self {
        Self { magnetic_field }
    }

    /// Create a zero reading.
    #[must_use]
    pub fn zero() -> Self {
        Self {
            magnetic_field: Vector3::zeros(),
        }
    }

    /// Get the field magnitude (Tesla).
    #[must_use]
    pub fn magnitude(&self) -> f64 {
        self.magnetic_field.norm()
    }

    /// Get the field magnitude in microtesla.
    #[must_use]
    pub fn magnitude_microtesla(&self) -> f64 {
        self.magnetic_field.norm() * 1e6
    }

    /// Compute the heading (yaw) angle from the magnetic field reading.
    ///
    /// This assumes the sensor is approximately level and returns the
    /// angle to magnetic north in radians, where:
    /// - 0 = North (positive Y in sensor frame)
    /// - π/2 = East (positive X in sensor frame)
    ///
    /// For tilted sensors, you should use a tilt-compensated algorithm
    /// that incorporates accelerometer data.
    #[must_use]
    pub fn heading(&self) -> f64 {
        // atan2(east, north) = atan2(x, y)
        self.magnetic_field.x.atan2(self.magnetic_field.y)
    }

    /// Compute the heading in degrees.
    #[must_use]
    pub fn heading_degrees(&self) -> f64 {
        self.heading().to_degrees()
    }

    /// Convert to `SensorData` for generic handling.
    #[must_use]
    pub fn to_sensor_data(self) -> SensorData {
        SensorData::Magnetometer {
            magnetic_field: self.magnetic_field,
        }
    }

    /// Convert to `SensorObservation` for inclusion in simulation observations.
    #[must_use]
    pub fn to_sensor_observation(self, sensor_id: u64, body_id: BodyId) -> SensorObservation {
        SensorObservation::magnetometer(sensor_id, body_id, self.magnetic_field)
    }
}

impl Default for MagnetometerReading {
    fn default() -> Self {
        Self::zero()
    }
}

/// Magnetometer sensor.
///
/// Measures the magnetic field at the sensor location, primarily used
/// for compass-like heading estimation.
///
/// # Example
///
/// ```
/// use sim_sensor::{Magnetometer, MagnetometerConfig};
/// use sim_types::{BodyId, Pose};
/// use nalgebra::{Point3, Vector3, UnitQuaternion};
///
/// // Create a magnetometer on body 1
/// let sensor = Magnetometer::new(BodyId::new(1), MagnetometerConfig::default());
///
/// // Read at identity pose (facing +Y = North)
/// let pose = Pose::identity();
/// let reading = sensor.read(&pose);
///
/// // Heading should be close to 0 (North)
/// let heading = reading.heading_degrees();
/// println!("Heading: {:.1}°", heading);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Magnetometer {
    /// Unique sensor ID.
    id: SensorId,
    /// Body this sensor is attached to.
    body_id: BodyId,
    /// Optional name for debugging.
    name: Option<String>,
    /// Sensor configuration.
    config: MagnetometerConfig,
}

impl Magnetometer {
    /// Create a new magnetometer sensor.
    #[must_use]
    pub fn new(body_id: BodyId, config: MagnetometerConfig) -> Self {
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
    pub const fn id(&self) -> SensorId {
        self.id
    }

    /// Get the body ID this sensor is attached to.
    #[must_use]
    pub const fn body_id(&self) -> BodyId {
        self.body_id
    }

    /// Get the sensor name.
    #[must_use]
    pub fn name(&self) -> Option<&str> {
        self.name.as_deref()
    }

    /// Get the sensor configuration.
    #[must_use]
    pub fn config(&self) -> &MagnetometerConfig {
        &self.config
    }

    /// Get a mutable reference to the configuration.
    pub fn config_mut(&mut self) -> &mut MagnetometerConfig {
        &mut self.config
    }

    /// Set the sensor ID.
    pub fn set_id(&mut self, id: SensorId) {
        self.id = id;
    }

    /// Read the sensor from body pose.
    ///
    /// # Arguments
    ///
    /// * `body_pose` - Current pose of the body the sensor is attached to
    #[must_use]
    pub fn read(&self, body_pose: &Pose) -> MagnetometerReading {
        self.read_with_local_field(body_pose, Vector3::zeros())
    }

    /// Read the sensor with an additional local magnetic field.
    ///
    /// This allows simulating local magnetic disturbances from motors,
    /// magnets, or other sources.
    ///
    /// # Arguments
    ///
    /// * `body_pose` - Current pose of the body the sensor is attached to
    /// * `local_field` - Additional magnetic field in world frame (Tesla)
    #[must_use]
    pub fn read_with_local_field(
        &self,
        body_pose: &Pose,
        local_field: Vector3<f64>,
    ) -> MagnetometerReading {
        // Total field in world frame
        let total_field_world = self.config.earth_field + local_field;

        // Transform to body frame
        let field_body = body_pose.inverse_transform_vector(&total_field_world);

        // Transform to sensor frame
        let field_sensor = self.config.local_rotation.inverse() * field_body;

        // Apply soft-iron distortion
        let field_distorted = self.config.soft_iron_matrix * field_sensor;

        // Apply hard-iron bias
        let field_with_bias = field_distorted + self.config.hard_iron_bias;

        MagnetometerReading::new(field_with_bias)
    }

    /// Read and return a generic [`SensorReading`].
    #[must_use]
    pub fn read_as_sensor_reading(&self, body_pose: &Pose, timestamp: f64) -> SensorReading {
        let reading = self.read(body_pose);
        SensorReading::new(
            self.id,
            SensorType::Magnetometer,
            timestamp,
            reading.to_sensor_data(),
        )
    }

    /// Read and return a `SensorObservation` for inclusion in simulation observations.
    #[must_use]
    pub fn read_as_observation(&self, body_pose: &Pose) -> SensorObservation {
        let reading = self.read(body_pose);
        let mut obs = reading.to_sensor_observation(self.id.raw(), self.body_id);
        if let Some(name) = &self.name {
            obs = obs.with_name(name.clone());
        }
        obs
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use nalgebra::{Point3, UnitQuaternion};
    use std::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn test_config_default() {
        let config = MagnetometerConfig::default();
        // Check that we have a reasonable Earth field magnitude (~50 μT)
        let magnitude_ut = config.earth_field.norm() * 1e6;
        assert!(magnitude_ut > 30.0 && magnitude_ut < 70.0);
    }

    #[test]
    fn test_config_builder() {
        let field = Vector3::new(0.0, 50e-6, -30e-6);
        let config = MagnetometerConfig::new()
            .with_earth_field(field)
            .with_noise(1e-6);

        assert_eq!(config.earth_field, field);
        assert_eq!(config.noise_std, 1e-6);
    }

    #[test]
    fn test_config_for_location() {
        // Test a location with 45° inclination, 0° declination, 50 μT intensity
        let config = MagnetometerConfig::for_location(0.0, PI / 4.0, 50e-6);

        // At 45° inclination, horizontal and vertical components should be equal
        let horiz = config.earth_field.x.hypot(config.earth_field.y);
        let vert = config.earth_field.z.abs();

        assert_relative_eq!(horiz, vert, epsilon = 1e-10);
    }

    #[test]
    fn test_reading_magnitude() {
        let reading = MagnetometerReading::new(Vector3::new(30e-6, 40e-6, 0.0));
        assert_relative_eq!(reading.magnitude_microtesla(), 50.0, epsilon = 0.001);
    }

    #[test]
    fn test_reading_heading() {
        // Field pointing North (positive Y)
        let reading = MagnetometerReading::new(Vector3::new(0.0, 1.0, 0.0));
        assert_relative_eq!(reading.heading(), 0.0, epsilon = 0.001);

        // Field pointing East (positive X)
        let reading = MagnetometerReading::new(Vector3::new(1.0, 0.0, 0.0));
        assert_relative_eq!(reading.heading(), FRAC_PI_2, epsilon = 0.001);

        // Field pointing South (negative Y)
        let reading = MagnetometerReading::new(Vector3::new(0.0, -1.0, 0.0));
        assert_relative_eq!(reading.heading().abs(), PI, epsilon = 0.001);

        // Field pointing West (negative X)
        let reading = MagnetometerReading::new(Vector3::new(-1.0, 0.0, 0.0));
        assert_relative_eq!(reading.heading(), -FRAC_PI_2, epsilon = 0.001);
    }

    #[test]
    fn test_sensor_at_identity() {
        // Simple field pointing North only
        let config = MagnetometerConfig::new().with_earth_field(Vector3::new(0.0, 50e-6, 0.0));

        let sensor = Magnetometer::new(BodyId::new(1), config);
        let pose = Pose::identity();
        let reading = sensor.read(&pose);

        // At identity pose, sensor frame = world frame
        // Field should point North (positive Y)
        assert_relative_eq!(reading.magnetic_field.y, 50e-6, epsilon = 1e-10);
        assert_relative_eq!(reading.heading(), 0.0, epsilon = 0.001);
    }

    #[test]
    fn test_sensor_rotated_90_degrees() {
        // Simple field pointing North only
        let config = MagnetometerConfig::new().with_earth_field(Vector3::new(0.0, 50e-6, 0.0));

        let sensor = Magnetometer::new(BodyId::new(1), config);

        // Rotate body 90° counter-clockwise around Z (looking down)
        // from_euler_angles uses XYZ order: roll, pitch, yaw
        // Positive yaw rotates counter-clockwise when looking down
        // Body's X now points North (Y in world)
        // Body's Y now points West (-X in world)
        let rot = UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2);
        let pose = Pose::from_position_rotation(Point3::origin(), rot);
        let reading = sensor.read(&pose);

        // Earth field (North = +Y in world) should now appear as +X in body frame
        // when body is rotated 90° CCW around Z
        assert_relative_eq!(reading.magnetic_field.x, 50e-6, epsilon = 1e-9);
        assert_relative_eq!(reading.magnetic_field.y, 0.0, epsilon = 1e-9);

        // Heading should be 90° (East) since field appears to come from East in body frame
        assert_relative_eq!(reading.heading(), FRAC_PI_2, epsilon = 0.001);
    }

    #[test]
    fn test_sensor_hard_iron_bias() {
        let config = MagnetometerConfig::new()
            .with_earth_field(Vector3::new(0.0, 50e-6, 0.0))
            .with_hard_iron_bias(Vector3::new(10e-6, 0.0, 0.0));

        let sensor = Magnetometer::new(BodyId::new(1), config);
        let pose = Pose::identity();
        let reading = sensor.read(&pose);

        // Should have Earth field + bias
        assert_relative_eq!(reading.magnetic_field.x, 10e-6, epsilon = 1e-10);
        assert_relative_eq!(reading.magnetic_field.y, 50e-6, epsilon = 1e-10);
    }

    #[test]
    fn test_sensor_soft_iron_distortion() {
        // Soft iron that scales X by 2
        let soft_iron = Matrix3::from_diagonal(&Vector3::new(2.0, 1.0, 1.0));
        let config = MagnetometerConfig::new()
            .with_earth_field(Vector3::new(20e-6, 50e-6, 0.0))
            .with_soft_iron_matrix(soft_iron);

        let sensor = Magnetometer::new(BodyId::new(1), config);
        let pose = Pose::identity();
        let reading = sensor.read(&pose);

        // X should be doubled
        assert_relative_eq!(reading.magnetic_field.x, 40e-6, epsilon = 1e-10);
        assert_relative_eq!(reading.magnetic_field.y, 50e-6, epsilon = 1e-10);
    }

    #[test]
    fn test_sensor_local_field() {
        let config = MagnetometerConfig::new().with_earth_field(Vector3::new(0.0, 50e-6, 0.0));

        let sensor = Magnetometer::new(BodyId::new(1), config);
        let pose = Pose::identity();

        // Add a local field from, say, a motor
        let local_field = Vector3::new(10e-6, 0.0, 0.0);
        let reading = sensor.read_with_local_field(&pose, local_field);

        // Should have combined field
        assert_relative_eq!(reading.magnetic_field.x, 10e-6, epsilon = 1e-10);
        assert_relative_eq!(reading.magnetic_field.y, 50e-6, epsilon = 1e-10);
    }

    #[test]
    fn test_reading_to_sensor_data() {
        let reading = MagnetometerReading::new(Vector3::new(30e-6, 40e-6, 10e-6));
        let data = reading.to_sensor_data();

        match data {
            SensorData::Magnetometer { magnetic_field } => {
                assert_eq!(magnetic_field, Vector3::new(30e-6, 40e-6, 10e-6));
            }
            _ => panic!("Expected Magnetometer data"),
        }
    }

    #[test]
    fn test_sensor_reading_output() {
        let sensor = Magnetometer::new(BodyId::new(1), MagnetometerConfig::default())
            .with_id(SensorId::new(42));

        let pose = Pose::identity();
        let reading = sensor.read_as_sensor_reading(&pose, 0.1);

        assert_eq!(reading.sensor_id.raw(), 42);
        assert_eq!(reading.sensor_type, SensorType::Magnetometer);
        assert_eq!(reading.timestamp, 0.1);
    }
}
