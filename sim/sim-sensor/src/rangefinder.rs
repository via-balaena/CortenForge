//! Rangefinder sensor for distance measurement.
//!
//! A rangefinder casts a ray from its position in a specified direction and
//! measures the distance to the first intersected surface. This is commonly
//! used for:
//!
//! - Obstacle detection and avoidance
//! - Height above ground measurement
//! - Proximity sensing
//!
//! The sensor can be configured with:
//! - Ray direction (in body frame)
//! - Maximum range (returns `max_range` if nothing is hit)
//! - Minimum range (returns `min_range` if closer)
//! - Noise characteristics

use nalgebra::{Point3, UnitVector3, Vector3};
use sim_types::{BodyId, Pose, SensorObservation};

use crate::{SensorData, SensorId, SensorReading, SensorType};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for a rangefinder sensor.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RangefinderConfig {
    /// Position of the sensor in the body's local frame.
    pub local_position: Vector3<f64>,

    /// Direction of the ray in the body's local frame.
    /// This will be normalized internally.
    pub local_direction: Vector3<f64>,

    /// Minimum detectable range (meters).
    /// Objects closer than this will read as `min_range`.
    pub min_range: f64,

    /// Maximum detectable range (meters).
    /// Objects farther than this (or no hit) will read as `max_range`.
    pub max_range: f64,

    /// Field of view angle (radians).
    /// A narrow beam (0) is ideal; larger values simulate wider beams.
    /// This is currently informational - the actual ray is a single line.
    pub beam_width: f64,

    /// Noise standard deviation (meters).
    /// Set to 0 for an ideal sensor.
    pub noise_std: f64,

    /// Whether to return `f64::INFINITY` for no hit instead of `max_range`.
    pub infinity_on_no_hit: bool,
}

impl Default for RangefinderConfig {
    fn default() -> Self {
        Self {
            local_position: Vector3::zeros(),
            local_direction: -Vector3::z(), // Default: pointing down
            min_range: 0.01,                // 1 cm
            max_range: 10.0,                // 10 m
            beam_width: 0.0,                // Ideal narrow beam
            noise_std: 0.0,                 // No noise
            infinity_on_no_hit: false,
        }
    }
}

impl RangefinderConfig {
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

    /// Set the ray direction in the body's local frame.
    #[must_use]
    pub fn with_local_direction(mut self, direction: Vector3<f64>) -> Self {
        self.local_direction = direction;
        self
    }

    /// Set the minimum range.
    #[must_use]
    pub fn with_min_range(mut self, min_range: f64) -> Self {
        self.min_range = min_range;
        self
    }

    /// Set the maximum range.
    #[must_use]
    pub fn with_max_range(mut self, max_range: f64) -> Self {
        self.max_range = max_range;
        self
    }

    /// Set the beam width.
    #[must_use]
    pub fn with_beam_width(mut self, beam_width: f64) -> Self {
        self.beam_width = beam_width;
        self
    }

    /// Set the noise standard deviation.
    #[must_use]
    pub fn with_noise(mut self, std: f64) -> Self {
        self.noise_std = std;
        self
    }

    /// Set whether to return infinity on no hit.
    #[must_use]
    pub fn with_infinity_on_no_hit(mut self, infinity: bool) -> Self {
        self.infinity_on_no_hit = infinity;
        self
    }

    /// Create a downward-facing height sensor.
    #[must_use]
    pub fn height_sensor(max_height: f64) -> Self {
        Self {
            local_direction: -Vector3::z(),
            max_range: max_height,
            ..Self::default()
        }
    }

    /// Create a forward-facing proximity sensor.
    #[must_use]
    pub fn forward_proximity(max_range: f64) -> Self {
        Self {
            local_direction: Vector3::x(),
            max_range,
            ..Self::default()
        }
    }

    /// Create a LIDAR-style sensor (narrow beam, long range).
    #[must_use]
    pub fn lidar(max_range: f64) -> Self {
        Self {
            local_direction: Vector3::x(),
            max_range,
            min_range: 0.1,
            beam_width: 0.001, // ~0.06 degrees
            ..Self::default()
        }
    }

    /// Create an ultrasonic-style sensor (wider beam, shorter range).
    #[must_use]
    pub fn ultrasonic(max_range: f64) -> Self {
        Self {
            local_direction: Vector3::x(),
            max_range,
            min_range: 0.02,
            beam_width: 0.5, // ~30 degrees cone
            noise_std: 0.01, // 1cm noise typical
            ..Self::default()
        }
    }
}

/// Rangefinder reading containing measured distance.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RangefinderReading {
    /// Measured distance (meters).
    /// May be clamped to `min_range`/`max_range`, or infinity if configured.
    pub distance: f64,

    /// Whether a surface was hit within the valid range.
    pub hit: bool,

    /// Ray origin in world frame (for debugging/visualization).
    pub ray_origin: Point3<f64>,

    /// Ray direction in world frame (for debugging/visualization).
    pub ray_direction: Vector3<f64>,
}

impl RangefinderReading {
    /// Create a new reading with a hit.
    #[must_use]
    pub fn with_hit(distance: f64, ray_origin: Point3<f64>, ray_direction: Vector3<f64>) -> Self {
        Self {
            distance,
            hit: true,
            ray_origin,
            ray_direction,
        }
    }

    /// Create a reading with no hit.
    #[must_use]
    pub fn no_hit(
        max_range_or_infinity: f64,
        ray_origin: Point3<f64>,
        ray_direction: Vector3<f64>,
    ) -> Self {
        Self {
            distance: max_range_or_infinity,
            hit: false,
            ray_origin,
            ray_direction,
        }
    }

    /// Check if the sensor detected a surface.
    #[must_use]
    pub fn is_hit(&self) -> bool {
        self.hit
    }

    /// Get the hit point in world coordinates (if hit).
    #[must_use]
    pub fn hit_point(&self) -> Option<Point3<f64>> {
        if self.hit {
            Some(self.ray_origin + self.ray_direction * self.distance)
        } else {
            None
        }
    }

    /// Convert to `SensorData` for generic handling.
    #[must_use]
    pub fn to_sensor_data(self) -> SensorData {
        SensorData::Rangefinder {
            distance: self.distance,
            hit: self.hit,
        }
    }

    /// Convert to `SensorObservation` for inclusion in simulation observations.
    #[must_use]
    pub fn to_sensor_observation(self, sensor_id: u64, body_id: BodyId) -> SensorObservation {
        SensorObservation::rangefinder(sensor_id, body_id, self.distance, self.hit)
    }
}

impl Default for RangefinderReading {
    fn default() -> Self {
        Self {
            distance: f64::INFINITY,
            hit: false,
            ray_origin: Point3::origin(),
            ray_direction: -Vector3::z(),
        }
    }
}

/// A ray cast result for the rangefinder.
#[derive(Debug, Clone, Copy)]
pub struct RayHit {
    /// Distance from ray origin to hit point.
    pub distance: f64,
    /// Hit point in world coordinates.
    pub point: Point3<f64>,
    /// Surface normal at hit point.
    pub normal: Vector3<f64>,
    /// ID of the body that was hit (if any).
    pub body_id: Option<BodyId>,
}

/// Trait for objects that can perform ray casting.
///
/// Implement this trait for your world/scene type to enable rangefinder sensing.
pub trait RayCaster {
    /// Cast a ray and return the closest hit, if any.
    ///
    /// # Arguments
    ///
    /// * `origin` - Ray origin in world coordinates
    /// * `direction` - Ray direction (unit vector) in world coordinates
    /// * `max_distance` - Maximum distance to check
    /// * `exclude_body` - Body to exclude from ray casting (the sensor's body)
    ///
    /// # Returns
    ///
    /// The closest ray hit, or `None` if nothing was hit within `max_distance`.
    fn cast_ray(
        &self,
        origin: Point3<f64>,
        direction: UnitVector3<f64>,
        max_distance: f64,
        exclude_body: Option<BodyId>,
    ) -> Option<RayHit>;
}

/// Rangefinder sensor.
///
/// Measures distance by casting a ray and detecting the first intersection.
///
/// # Example
///
/// ```
/// use sim_sensor::{Rangefinder, RangefinderConfig, RangefinderReading};
/// use sim_types::{BodyId, Pose};
/// use nalgebra::{Point3, Vector3};
///
/// // Create a downward-facing height sensor
/// let sensor = Rangefinder::new(BodyId::new(1), RangefinderConfig::height_sensor(10.0));
///
/// // Read with a known distance (e.g., from a ray cast)
/// let body_pose = Pose::from_position(Point3::new(0.0, 0.0, 1.5));
/// let reading = sensor.read_with_distance(&body_pose, Some(1.5));
///
/// assert!(reading.is_hit());
/// assert!((reading.distance - 1.5).abs() < 0.001);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Rangefinder {
    /// Unique sensor ID.
    id: SensorId,
    /// Body this sensor is attached to.
    body_id: BodyId,
    /// Optional name for debugging.
    name: Option<String>,
    /// Sensor configuration.
    config: RangefinderConfig,
}

impl Rangefinder {
    /// Create a new rangefinder sensor.
    #[must_use]
    pub fn new(body_id: BodyId, config: RangefinderConfig) -> Self {
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
    pub fn config(&self) -> &RangefinderConfig {
        &self.config
    }

    /// Get a mutable reference to the configuration.
    pub fn config_mut(&mut self) -> &mut RangefinderConfig {
        &mut self.config
    }

    /// Set the sensor ID.
    pub fn set_id(&mut self, id: SensorId) {
        self.id = id;
    }

    /// Get the ray origin in world coordinates for a given body pose.
    #[must_use]
    pub fn ray_origin(&self, body_pose: &Pose) -> Point3<f64> {
        body_pose.transform_point(&Point3::from(self.config.local_position))
    }

    /// Get the ray direction in world coordinates for a given body pose.
    #[must_use]
    pub fn ray_direction(&self, body_pose: &Pose) -> Vector3<f64> {
        body_pose
            .transform_vector(&self.config.local_direction)
            .normalize()
    }

    /// Read the sensor with a known distance (e.g., from external ray casting).
    ///
    /// # Arguments
    ///
    /// * `body_pose` - Current pose of the body the sensor is attached to
    /// * `raw_distance` - Distance from ray cast, or `None` if no hit
    #[must_use]
    pub fn read_with_distance(
        &self,
        body_pose: &Pose,
        raw_distance: Option<f64>,
    ) -> RangefinderReading {
        let ray_origin = self.ray_origin(body_pose);
        let ray_direction = self.ray_direction(body_pose);

        raw_distance.map_or_else(
            || {
                let no_hit_value = if self.config.infinity_on_no_hit {
                    f64::INFINITY
                } else {
                    self.config.max_range
                };
                RangefinderReading::no_hit(no_hit_value, ray_origin, ray_direction)
            },
            |dist| {
                // Clamp to valid range
                let clamped = dist.clamp(self.config.min_range, self.config.max_range);
                RangefinderReading::with_hit(clamped, ray_origin, ray_direction)
            },
        )
    }

    /// Read the sensor using a ray caster.
    ///
    /// This performs the actual ray cast against the environment.
    #[must_use]
    pub fn read<R: RayCaster>(&self, body_pose: &Pose, ray_caster: &R) -> RangefinderReading {
        let ray_origin = self.ray_origin(body_pose);
        let ray_dir = self.ray_direction(body_pose);

        // Safely create unit vector
        let Some(unit_dir) = UnitVector3::try_new(ray_dir, 1e-10) else {
            // Invalid direction, return no hit
            return RangefinderReading::no_hit(
                if self.config.infinity_on_no_hit {
                    f64::INFINITY
                } else {
                    self.config.max_range
                },
                ray_origin,
                ray_dir,
            );
        };

        let hit = ray_caster.cast_ray(
            ray_origin,
            unit_dir,
            self.config.max_range,
            Some(self.body_id),
        );

        match hit {
            Some(ray_hit) if ray_hit.distance >= self.config.min_range => {
                RangefinderReading::with_hit(ray_hit.distance, ray_origin, ray_dir)
            }
            Some(_) => {
                // Hit is too close, clamp to min_range
                RangefinderReading::with_hit(self.config.min_range, ray_origin, ray_dir)
            }
            None => {
                let no_hit_value = if self.config.infinity_on_no_hit {
                    f64::INFINITY
                } else {
                    self.config.max_range
                };
                RangefinderReading::no_hit(no_hit_value, ray_origin, ray_dir)
            }
        }
    }

    /// Read and return a generic [`SensorReading`].
    #[must_use]
    pub fn read_as_sensor_reading(
        &self,
        body_pose: &Pose,
        raw_distance: Option<f64>,
        timestamp: f64,
    ) -> SensorReading {
        let reading = self.read_with_distance(body_pose, raw_distance);
        SensorReading::new(
            self.id,
            SensorType::Rangefinder,
            timestamp,
            reading.to_sensor_data(),
        )
    }

    /// Read and return a `SensorObservation` for inclusion in simulation observations.
    #[must_use]
    pub fn read_as_observation(
        &self,
        body_pose: &Pose,
        raw_distance: Option<f64>,
    ) -> SensorObservation {
        let reading = self.read_with_distance(body_pose, raw_distance);
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
    use nalgebra::UnitQuaternion;

    #[test]
    fn test_config_default() {
        let config = RangefinderConfig::default();
        assert_eq!(config.max_range, 10.0);
        assert_eq!(config.min_range, 0.01);
        assert!(!config.infinity_on_no_hit);
    }

    #[test]
    fn test_config_builder() {
        let config = RangefinderConfig::new()
            .with_max_range(20.0)
            .with_min_range(0.1)
            .with_local_direction(Vector3::x())
            .with_noise(0.01);

        assert_eq!(config.max_range, 20.0);
        assert_eq!(config.min_range, 0.1);
        assert_eq!(config.local_direction, Vector3::x());
        assert_eq!(config.noise_std, 0.01);
    }

    #[test]
    fn test_config_presets() {
        let height = RangefinderConfig::height_sensor(5.0);
        assert_eq!(height.local_direction, -Vector3::z());
        assert_eq!(height.max_range, 5.0);

        let forward = RangefinderConfig::forward_proximity(2.0);
        assert_eq!(forward.local_direction, Vector3::x());
        assert_eq!(forward.max_range, 2.0);

        let lidar = RangefinderConfig::lidar(100.0);
        assert_eq!(lidar.max_range, 100.0);
        assert!(lidar.beam_width < 0.01);

        let ultrasonic = RangefinderConfig::ultrasonic(4.0);
        assert!(ultrasonic.beam_width > 0.1);
        assert!(ultrasonic.noise_std > 0.0);
    }

    #[test]
    fn test_reading_with_hit() {
        let ray_origin = Point3::new(0.0, 0.0, 1.0);
        let ray_direction = -Vector3::z();
        let reading = RangefinderReading::with_hit(0.5, ray_origin, ray_direction);

        assert!(reading.is_hit());
        assert_eq!(reading.distance, 0.5);

        let hit_point = reading.hit_point().expect("should have hit point");
        assert_relative_eq!(hit_point.z, 0.5, epsilon = 0.001);
    }

    #[test]
    fn test_reading_no_hit() {
        let ray_origin = Point3::origin();
        let ray_direction = -Vector3::z();
        let reading = RangefinderReading::no_hit(10.0, ray_origin, ray_direction);

        assert!(!reading.is_hit());
        assert_eq!(reading.distance, 10.0);
        assert!(reading.hit_point().is_none());
    }

    #[test]
    fn test_sensor_ray_origin_and_direction() {
        let config = RangefinderConfig::new()
            .with_local_position(Vector3::new(0.1, 0.0, 0.0))
            .with_local_direction(-Vector3::z());

        let sensor = Rangefinder::new(BodyId::new(1), config);

        // At identity pose
        let pose = Pose::identity();
        let origin = sensor.ray_origin(&pose);
        let direction = sensor.ray_direction(&pose);

        assert_relative_eq!(origin.x, 0.1, epsilon = 0.001);
        assert_relative_eq!(direction.z, -1.0, epsilon = 0.001);

        // Rotated 90 degrees around Y
        // Using from_euler_angles(roll, pitch, yaw) = from_euler_angles(0, PI/2, 0)
        // This rotates: X -> -Z, Z -> X
        // So -Z in body frame becomes -X in world frame
        let rot = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0);
        let rotated_pose = Pose::from_position_rotation(Point3::origin(), rot);
        let rotated_dir = sensor.ray_direction(&rotated_pose);

        // -Z in body frame becomes -X in world frame after 90° pitch rotation
        assert_relative_eq!(rotated_dir.x, -1.0, epsilon = 0.001);
    }

    #[test]
    fn test_sensor_read_with_distance() {
        let sensor = Rangefinder::new(BodyId::new(1), RangefinderConfig::default());
        let pose = Pose::identity();

        // Normal reading
        let reading = sensor.read_with_distance(&pose, Some(5.0));
        assert!(reading.is_hit());
        assert_eq!(reading.distance, 5.0);

        // Clamped to max
        let reading = sensor.read_with_distance(&pose, Some(15.0));
        assert!(reading.is_hit());
        assert_eq!(reading.distance, 10.0); // max_range

        // Clamped to min
        let reading = sensor.read_with_distance(&pose, Some(0.001));
        assert!(reading.is_hit());
        assert_eq!(reading.distance, 0.01); // min_range

        // No hit
        let reading = sensor.read_with_distance(&pose, None);
        assert!(!reading.is_hit());
        assert_eq!(reading.distance, 10.0); // max_range
    }

    #[test]
    fn test_sensor_infinity_on_no_hit() {
        let config = RangefinderConfig::new().with_infinity_on_no_hit(true);
        let sensor = Rangefinder::new(BodyId::new(1), config);
        let pose = Pose::identity();

        let reading = sensor.read_with_distance(&pose, None);
        assert!(!reading.is_hit());
        assert!(reading.distance.is_infinite());
    }

    #[test]
    fn test_reading_to_sensor_data() {
        let reading = RangefinderReading::with_hit(2.5, Point3::origin(), -Vector3::z());
        let data = reading.to_sensor_data();

        match data {
            SensorData::Rangefinder { distance, hit } => {
                assert_eq!(distance, 2.5);
                assert!(hit);
            }
            _ => panic!("Expected Rangefinder data"),
        }
    }

    #[test]
    fn test_sensor_reading_output() {
        let sensor = Rangefinder::new(BodyId::new(1), RangefinderConfig::default())
            .with_id(SensorId::new(42));

        let pose = Pose::identity();
        let reading = sensor.read_as_sensor_reading(&pose, Some(3.0), 0.1);

        assert_eq!(reading.sensor_id.raw(), 42);
        assert_eq!(reading.sensor_type, SensorType::Rangefinder);
        assert_eq!(reading.timestamp, 0.1);
    }

    // Simple ray caster for testing
    struct TestRayCaster {
        hit_distance: Option<f64>,
    }

    impl RayCaster for TestRayCaster {
        fn cast_ray(
            &self,
            _origin: Point3<f64>,
            _direction: UnitVector3<f64>,
            max_distance: f64,
            _exclude_body: Option<BodyId>,
        ) -> Option<RayHit> {
            self.hit_distance
                .filter(|&d| d <= max_distance)
                .map(|d| RayHit {
                    distance: d,
                    point: Point3::origin(), // Not used in tests
                    normal: Vector3::z(),
                    body_id: None,
                })
        }
    }

    #[test]
    fn test_sensor_read_with_raycaster() {
        let sensor = Rangefinder::new(BodyId::new(1), RangefinderConfig::default());
        let pose = Pose::identity();

        // Hit at 5m
        let caster = TestRayCaster {
            hit_distance: Some(5.0),
        };
        let reading = sensor.read(&pose, &caster);
        assert!(reading.is_hit());
        assert_eq!(reading.distance, 5.0);

        // No hit
        let caster = TestRayCaster { hit_distance: None };
        let reading = sensor.read(&pose, &caster);
        assert!(!reading.is_hit());
        assert_eq!(reading.distance, 10.0);

        // Hit beyond max range
        let caster = TestRayCaster {
            hit_distance: Some(15.0),
        };
        let reading = sensor.read(&pose, &caster);
        assert!(!reading.is_hit()); // Beyond max_distance in cast_ray
    }
}
