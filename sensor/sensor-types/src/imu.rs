//! Inertial Measurement Unit (IMU) data types.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::{CoordinateFrame, Timestamp};

/// A reading from an Inertial Measurement Unit (IMU).
///
/// Contains acceleration and angular velocity data, typically sampled
/// at high rates (100Hz - 1kHz).
///
/// # Units
///
/// - Acceleration: meters per second squared (m/s²)
/// - Angular velocity: radians per second (rad/s)
///
/// # Example
///
/// ```
/// use sensor_types::{ImuReading, CoordinateFrame, Timestamp};
///
/// let reading = ImuReading {
///     timestamp: Timestamp::from_secs_f64(0.001),
///     acceleration: [0.0, 0.0, 9.81],  // Gravity
///     angular_velocity: [0.0, 0.0, 0.1],  // Slight yaw rate
///     frame: CoordinateFrame::Body,
/// };
///
/// assert!(reading.acceleration[2] > 9.0);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImuReading {
    /// Timestamp of the reading.
    pub timestamp: Timestamp,

    /// Linear acceleration in m/s²: `[x, y, z]`.
    ///
    /// In the body frame, this typically includes gravity.
    /// A stationary IMU aligned with gravity will read approximately
    /// `[0, 0, 9.81]` (assuming Z-up convention).
    pub acceleration: [f64; 3],

    /// Angular velocity in rad/s: `[roll_rate, pitch_rate, yaw_rate]`.
    ///
    /// Also known as gyroscope data.
    pub angular_velocity: [f64; 3],

    /// Coordinate frame of the measurement.
    pub frame: CoordinateFrame,
}

impl ImuReading {
    /// Creates a new IMU reading.
    #[must_use]
    pub const fn new(
        timestamp: Timestamp,
        acceleration: [f64; 3],
        angular_velocity: [f64; 3],
        frame: CoordinateFrame,
    ) -> Self {
        Self {
            timestamp,
            acceleration,
            angular_velocity,
            frame,
        }
    }

    /// Creates a zero IMU reading (no acceleration, no rotation).
    #[must_use]
    pub const fn zero(timestamp: Timestamp) -> Self {
        Self {
            timestamp,
            acceleration: [0.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.0],
            frame: CoordinateFrame::Body,
        }
    }

    /// Returns the magnitude of the acceleration vector.
    ///
    /// For a stationary sensor, this should be approximately 9.81 m/s².
    #[must_use]
    pub fn acceleration_magnitude(&self) -> f64 {
        let [x, y, z] = self.acceleration;
        x.hypot(y).hypot(z)
    }

    /// Returns the magnitude of the angular velocity vector.
    #[must_use]
    pub fn angular_velocity_magnitude(&self) -> f64 {
        let [x, y, z] = self.angular_velocity;
        x.hypot(y).hypot(z)
    }

    /// Checks if the sensor is approximately stationary.
    ///
    /// Returns `true` if angular velocity is below the threshold.
    #[must_use]
    pub fn is_stationary(&self, angular_threshold: f64) -> bool {
        self.angular_velocity_magnitude() < angular_threshold
    }

    /// Returns the gravity-subtracted acceleration (assuming Z-up).
    ///
    /// This assumes the sensor is aligned with gravity pointing in -Z.
    #[must_use]
    pub fn linear_acceleration(&self, gravity: f64) -> [f64; 3] {
        [
            self.acceleration[0],
            self.acceleration[1],
            self.acceleration[2] - gravity,
        ]
    }
}

impl Default for ImuReading {
    fn default() -> Self {
        Self::zero(Timestamp::zero())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_new() {
        let reading = ImuReading::new(
            Timestamp::from_secs_f64(1.0),
            [0.0, 0.0, 9.81],
            [0.0, 0.0, 0.1],
            CoordinateFrame::Body,
        );

        assert!((reading.acceleration[2] - 9.81).abs() < 1e-10);
        assert!((reading.angular_velocity[2] - 0.1).abs() < 1e-10);
    }

    #[test]
    fn imu_zero() {
        let reading = ImuReading::zero(Timestamp::from_nanos(1000));
        assert_eq!(reading.acceleration, [0.0, 0.0, 0.0]);
        assert_eq!(reading.angular_velocity, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn acceleration_magnitude() {
        let reading = ImuReading {
            timestamp: Timestamp::zero(),
            acceleration: [0.0, 0.0, 9.81],
            angular_velocity: [0.0, 0.0, 0.0],
            frame: CoordinateFrame::Body,
        };

        assert!((reading.acceleration_magnitude() - 9.81).abs() < 1e-10);
    }

    #[test]
    fn angular_velocity_magnitude() {
        let reading = ImuReading {
            timestamp: Timestamp::zero(),
            acceleration: [0.0, 0.0, 0.0],
            angular_velocity: [0.1, 0.0, 0.0],
            frame: CoordinateFrame::Body,
        };

        assert!((reading.angular_velocity_magnitude() - 0.1).abs() < 1e-10);
    }

    #[test]
    fn is_stationary() {
        let stationary = ImuReading {
            timestamp: Timestamp::zero(),
            acceleration: [0.0, 0.0, 9.81],
            angular_velocity: [0.001, 0.001, 0.001],
            frame: CoordinateFrame::Body,
        };

        let moving = ImuReading {
            timestamp: Timestamp::zero(),
            acceleration: [0.0, 0.0, 9.81],
            angular_velocity: [0.5, 0.0, 0.0],
            frame: CoordinateFrame::Body,
        };

        assert!(stationary.is_stationary(0.01));
        assert!(!moving.is_stationary(0.01));
    }

    #[test]
    fn linear_acceleration() {
        let reading = ImuReading {
            timestamp: Timestamp::zero(),
            acceleration: [1.0, 2.0, 12.81],
            angular_velocity: [0.0, 0.0, 0.0],
            frame: CoordinateFrame::Body,
        };

        let linear = reading.linear_acceleration(9.81);
        assert!((linear[0] - 1.0).abs() < 1e-10);
        assert!((linear[1] - 2.0).abs() < 1e-10);
        assert!((linear[2] - 3.0).abs() < 1e-10);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn imu_serialization() {
        let reading = ImuReading::new(
            Timestamp::from_secs_f64(1.0),
            [0.0, 0.0, 9.81],
            [0.0, 0.0, 0.0],
            CoordinateFrame::Body,
        );

        let json = serde_json::to_string(&reading).ok();
        assert!(json.is_some());

        let parsed: Result<ImuReading, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }
}
