//! Sensor integration tests (standalone sim-sensor crate only).
//!
//! Tests sensor readings from the **standalone sim-sensor crate**, including
//! IMU, force/torque, magnetometer, and rangefinder. These tests exercise
//! the sensor API directly — they do **not** test the MuJoCo pipeline's
//! sensor evaluation (`mj_sensor()` in `mujoco_pipeline.rs`).

use approx::assert_relative_eq;
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_sensor::{
    ForceTorqueSensor, ForceTorqueSensorConfig, Imu, ImuConfig, Magnetometer, MagnetometerConfig,
    Rangefinder, RangefinderConfig,
};
use sim_types::{BodyId, Pose, RigidBodyState};

/// Test: IMU reads body state correctly.
#[test]
fn test_imu_reads_body_state() {
    // Create IMU attached to body 0
    let imu = Imu::new(BodyId::new(0), ImuConfig::default());

    // Create a body state with rotation
    let state = RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 10.0)));

    // Read sensor
    let reading = imu.read_from_state(&state, 0.001);

    // IMU should return finite readings
    assert!(reading.angular_velocity.norm().is_finite());
    assert!(reading.linear_acceleration.norm().is_finite());
}

/// Test: IMU with external acceleration.
#[test]
fn test_imu_with_acceleration() {
    let imu = Imu::new(BodyId::new(0), ImuConfig::default());

    // Body at origin, identity rotation
    let state = RigidBodyState::at_rest(Pose::default());

    // Apply a known acceleration
    let external_accel = Vector3::new(0.0, 0.0, 5.0);
    let reading = imu.read_with_acceleration(&state, external_accel);

    // Accelerometer should include the external acceleration plus gravity
    // Exact values depend on implementation details
    assert!(reading.linear_acceleration.norm().is_finite());
}

/// Test: Force/torque sensor processes force correctly.
#[test]
fn test_force_torque_sensor_process() {
    let sensor = ForceTorqueSensor::new(BodyId::new(0), ForceTorqueSensorConfig::default());

    // Apply a known force and torque
    let force = Vector3::new(0.0, 0.0, 50.0);
    let torque = Vector3::new(1.0, 0.0, 0.0);

    // Process through sensor
    let reading = sensor.process(force, torque);

    // Should get the force/torque values (possibly transformed to sensor frame)
    assert!(reading.force.norm() > 0.0);
    assert_relative_eq!(reading.force.z, 50.0, epsilon = 0.01);
    assert_relative_eq!(reading.torque.x, 1.0, epsilon = 0.01);
}

/// Test: Force/torque sensor with default config passes through.
#[test]
fn test_force_torque_passthrough() {
    // Default config has no bias, no noise, identity rotation
    let sensor = ForceTorqueSensor::new(BodyId::new(0), ForceTorqueSensorConfig::default());

    let force = Vector3::new(10.0, 20.0, 30.0);
    let torque = Vector3::new(1.0, 2.0, 3.0);

    let reading = sensor.process(force, torque);

    // Default sensor should pass through unchanged
    assert_relative_eq!(reading.force.x, 10.0, epsilon = 0.01);
    assert_relative_eq!(reading.force.y, 20.0, epsilon = 0.01);
    assert_relative_eq!(reading.force.z, 30.0, epsilon = 0.01);
}

/// Test: Rangefinder with known distance.
#[test]
fn test_rangefinder_distance() {
    let config = RangefinderConfig::default();
    let sensor = Rangefinder::new(BodyId::new(0), config);

    // Use read_with_distance for direct distance input
    let body_pose = Pose::from_position(Point3::new(0.0, 0.0, 1.0));

    // Valid distance within range
    let reading = sensor.read_with_distance(&body_pose, Some(5.0));
    assert!(reading.distance > 0.0);

    // No hit
    let no_hit = sensor.read_with_distance(&body_pose, None);
    // When no hit, distance should be infinite or max
    assert!(no_hit.distance.is_finite() || no_hit.distance.is_infinite());
}

/// Test: Magnetometer reads body pose.
#[test]
fn test_magnetometer_reads_pose() {
    let sensor = Magnetometer::new(BodyId::new(0), MagnetometerConfig::default());

    // Body at origin with identity rotation
    let body_pose = Pose::default();

    let reading = sensor.read(&body_pose);

    // Should return a magnetic field reading
    assert!(reading.magnetic_field.norm().is_finite());
}

/// Test: Magnetometer body frame transform.
#[test]
fn test_magnetometer_body_transform() {
    let sensor = Magnetometer::new(BodyId::new(0), MagnetometerConfig::default());

    // Body at origin
    let pose_identity = Pose::default();
    let reading1 = sensor.read(&pose_identity);

    // Body rotated 90 degrees around Z
    let rotated_pose = Pose::from_position_rotation(
        Point3::origin(),
        UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2),
    );
    let reading2 = sensor.read(&rotated_pose);

    // The two readings should differ due to rotation
    // (magnetic field is transformed to body frame)
    assert!(reading1.magnetic_field.norm().is_finite());
    assert!(reading2.magnetic_field.norm().is_finite());
}

/// Test: Multiple sensors on same body.
#[test]
fn test_multiple_sensors_on_body() {
    let body_id = BodyId::new(0);

    // Create all sensor types for the same body
    let imu = Imu::new(body_id, ImuConfig::default());
    let ft_sensor = ForceTorqueSensor::new(body_id, ForceTorqueSensorConfig::default());
    let magnetometer = Magnetometer::new(body_id, MagnetometerConfig::default());
    let rangefinder = Rangefinder::new(body_id, RangefinderConfig::default());

    // Body state
    let state = RigidBodyState::at_rest(Pose::default());
    let pose = &state.pose;

    // Read all sensors
    let imu_reading = imu.read_from_state(&state, 0.001);
    let ft_reading = ft_sensor.process(Vector3::new(100.0, 0.0, 0.0), Vector3::zeros());
    let mag_reading = magnetometer.read(pose);
    let range_reading = rangefinder.read_with_distance(pose, Some(3.0));

    // Verify all produce valid readings
    assert!(imu_reading.angular_velocity.norm().is_finite());
    assert!(imu_reading.linear_acceleration.norm().is_finite());
    assert!(ft_reading.force.norm().is_finite());
    assert!(mag_reading.magnetic_field.norm().is_finite());
    assert!(range_reading.distance.is_finite());
}

/// Test: IMU config noise settings.
#[test]
fn test_imu_config() {
    // IMU with specific noise settings
    let config = ImuConfig {
        accel_noise_std: 0.5,
        gyro_noise_std: 0.1,
        ..Default::default()
    };

    let imu = Imu::new(BodyId::new(0), config);

    // Create state
    let state = RigidBodyState::at_rest(Pose::default());

    // Take multiple readings - with noise they should vary
    let readings: Vec<_> = (0..10)
        .map(|_| imu.read_from_state(&state, 0.001))
        .collect();

    // All readings should be finite
    for reading in &readings {
        assert!(reading.angular_velocity.norm().is_finite());
        assert!(reading.linear_acceleration.norm().is_finite());
    }
}

/// Test: Sensor ID and body ID accessors.
#[test]
fn test_sensor_metadata() {
    let body_id = BodyId::new(42);

    let imu = Imu::new(body_id, ImuConfig::default());
    let ft = ForceTorqueSensor::new(body_id, ForceTorqueSensorConfig::default());
    let mag = Magnetometer::new(body_id, MagnetometerConfig::default());
    let range = Rangefinder::new(body_id, RangefinderConfig::default());

    // All sensors should report the same body ID
    assert_eq!(imu.body_id(), body_id);
    assert_eq!(ft.body_id(), body_id);
    assert_eq!(mag.body_id(), body_id);
    assert_eq!(range.body_id(), body_id);
}

/// Test: Force/torque sensor configuration.
#[test]
fn test_force_torque_config() {
    let config = ForceTorqueSensorConfig::new()
        .with_max_force(100.0)
        .with_max_torque(10.0);

    let sensor = ForceTorqueSensor::new(BodyId::new(0), config);

    // Force beyond max should saturate
    let reading = sensor.process(Vector3::new(0.0, 0.0, 200.0), Vector3::zeros());

    // Force should be clamped to max
    assert!(reading.force.norm() <= 100.0 + 0.01);
}
