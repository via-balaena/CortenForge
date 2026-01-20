//! Core sensor types and identifiers.

use nalgebra::Vector3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Unique identifier for a sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SensorId(pub u64);

impl SensorId {
    /// Create a new sensor ID.
    #[must_use]
    pub const fn new(id: u64) -> Self {
        Self(id)
    }

    /// Get the raw ID value.
    #[must_use]
    pub const fn raw(self) -> u64 {
        self.0
    }
}

impl From<u64> for SensorId {
    fn from(id: u64) -> Self {
        Self(id)
    }
}

impl std::fmt::Display for SensorId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Sensor({})", self.0)
    }
}

/// Type of sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum SensorType {
    /// Inertial measurement unit (accelerometer + gyroscope).
    Imu,
    /// Accelerometer only.
    Accelerometer,
    /// Gyroscope only.
    Gyroscope,
    /// Force/torque sensor (6-axis).
    ForceTorque,
    /// Touch/contact sensor.
    Touch,
}

impl std::fmt::Display for SensorType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Imu => write!(f, "IMU"),
            Self::Accelerometer => write!(f, "Accelerometer"),
            Self::Gyroscope => write!(f, "Gyroscope"),
            Self::ForceTorque => write!(f, "Force/Torque"),
            Self::Touch => write!(f, "Touch"),
        }
    }
}

/// A reading from a sensor at a specific time.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SensorReading {
    /// ID of the sensor that produced this reading.
    pub sensor_id: SensorId,
    /// Type of sensor.
    pub sensor_type: SensorType,
    /// Simulation time at which the reading was taken.
    pub timestamp: f64,
    /// The sensor data.
    pub data: SensorData,
}

impl SensorReading {
    /// Create a new sensor reading.
    #[must_use]
    pub fn new(
        sensor_id: SensorId,
        sensor_type: SensorType,
        timestamp: f64,
        data: SensorData,
    ) -> Self {
        Self {
            sensor_id,
            sensor_type,
            timestamp,
            data,
        }
    }
}

/// Data from a sensor reading.
///
/// Each variant corresponds to a specific sensor type and contains
/// the appropriate data fields.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum SensorData {
    /// IMU reading with linear acceleration and angular velocity.
    Imu {
        /// Linear acceleration in sensor frame (m/s^2).
        /// Includes gravity if `include_gravity` is true.
        linear_acceleration: Vector3<f64>,
        /// Angular velocity in sensor frame (rad/s).
        angular_velocity: Vector3<f64>,
    },

    /// Accelerometer reading (linear acceleration only).
    Accelerometer {
        /// Linear acceleration in sensor frame (m/s^2).
        linear_acceleration: Vector3<f64>,
    },

    /// Gyroscope reading (angular velocity only).
    Gyroscope {
        /// Angular velocity in sensor frame (rad/s).
        angular_velocity: Vector3<f64>,
    },

    /// Force/torque sensor reading.
    ForceTorque {
        /// Force measured at the sensor (N).
        force: Vector3<f64>,
        /// Torque measured at the sensor (Nm).
        torque: Vector3<f64>,
    },

    /// Touch sensor reading.
    Touch {
        /// Whether contact is detected.
        in_contact: bool,
        /// Number of contact points (if tracking multiple).
        contact_count: usize,
        /// Total contact force magnitude (N).
        contact_force: f64,
        /// Average contact normal (if in contact).
        contact_normal: Option<Vector3<f64>>,
    },
}

impl SensorData {
    /// Check if this is an IMU reading.
    #[must_use]
    pub fn is_imu(&self) -> bool {
        matches!(self, Self::Imu { .. })
    }

    /// Check if this is a force/torque reading.
    #[must_use]
    pub fn is_force_torque(&self) -> bool {
        matches!(self, Self::ForceTorque { .. })
    }

    /// Check if this is a touch reading.
    #[must_use]
    pub fn is_touch(&self) -> bool {
        matches!(self, Self::Touch { .. })
    }

    /// Get IMU data if this is an IMU reading.
    #[must_use]
    pub fn as_imu(&self) -> Option<(&Vector3<f64>, &Vector3<f64>)> {
        match self {
            Self::Imu {
                linear_acceleration,
                angular_velocity,
            } => Some((linear_acceleration, angular_velocity)),
            _ => None,
        }
    }

    /// Get force/torque data if this is a force/torque reading.
    #[must_use]
    pub fn as_force_torque(&self) -> Option<(&Vector3<f64>, &Vector3<f64>)> {
        match self {
            Self::ForceTorque { force, torque } => Some((force, torque)),
            _ => None,
        }
    }

    /// Get touch data if this is a touch reading.
    #[must_use]
    pub fn as_touch(&self) -> Option<(bool, usize, f64, Option<&Vector3<f64>>)> {
        match self {
            Self::Touch {
                in_contact,
                contact_count,
                contact_force,
                contact_normal,
            } => Some((
                *in_contact,
                *contact_count,
                *contact_force,
                contact_normal.as_ref(),
            )),
            _ => None,
        }
    }

    /// Convert to a flat f64 vector for ML consumption.
    ///
    /// Returns the data as a vector of f64 values suitable for neural network input.
    #[must_use]
    pub fn to_vec(&self) -> Vec<f64> {
        match self {
            Self::Imu {
                linear_acceleration,
                angular_velocity,
            } => {
                vec![
                    linear_acceleration.x,
                    linear_acceleration.y,
                    linear_acceleration.z,
                    angular_velocity.x,
                    angular_velocity.y,
                    angular_velocity.z,
                ]
            }
            Self::Accelerometer {
                linear_acceleration,
            } => {
                vec![
                    linear_acceleration.x,
                    linear_acceleration.y,
                    linear_acceleration.z,
                ]
            }
            Self::Gyroscope { angular_velocity } => {
                vec![angular_velocity.x, angular_velocity.y, angular_velocity.z]
            }
            Self::ForceTorque { force, torque } => {
                vec![force.x, force.y, force.z, torque.x, torque.y, torque.z]
            }
            Self::Touch {
                in_contact,
                contact_count,
                contact_force,
                contact_normal,
            } => {
                let mut v = vec![
                    if *in_contact { 1.0 } else { 0.0 },
                    *contact_count as f64,
                    *contact_force,
                ];
                if let Some(n) = contact_normal {
                    v.extend([n.x, n.y, n.z]);
                } else {
                    v.extend([0.0, 0.0, 0.0]);
                }
                v
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_id() {
        let id = SensorId::new(42);
        assert_eq!(id.raw(), 42);
        assert_eq!(id.to_string(), "Sensor(42)");

        let id2: SensorId = 42.into();
        assert_eq!(id, id2);
    }

    #[test]
    fn test_sensor_data_imu() {
        let data = SensorData::Imu {
            linear_acceleration: Vector3::new(0.0, 0.0, 9.81),
            angular_velocity: Vector3::new(0.1, 0.0, 0.0),
        };

        assert!(data.is_imu());
        assert!(!data.is_force_torque());
        assert!(!data.is_touch());

        let (accel, gyro) = data.as_imu().unwrap();
        assert!((accel.z - 9.81).abs() < 1e-10);
        assert!((gyro.x - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_sensor_data_to_vec() {
        let imu_data = SensorData::Imu {
            linear_acceleration: Vector3::new(1.0, 2.0, 3.0),
            angular_velocity: Vector3::new(4.0, 5.0, 6.0),
        };

        let vec = imu_data.to_vec();
        assert_eq!(vec.len(), 6);
        assert_eq!(vec, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_touch_data_to_vec() {
        let touch_data = SensorData::Touch {
            in_contact: true,
            contact_count: 3,
            contact_force: 10.5,
            contact_normal: Some(Vector3::new(0.0, 0.0, 1.0)),
        };

        let vec = touch_data.to_vec();
        assert_eq!(vec.len(), 6);
        assert_eq!(vec[0], 1.0); // in_contact = true
        assert_eq!(vec[1], 3.0); // contact_count
        assert_eq!(vec[2], 10.5); // contact_force
    }

    #[test]
    fn test_sensor_reading() {
        let reading = SensorReading::new(
            SensorId::new(1),
            SensorType::Imu,
            0.001,
            SensorData::Imu {
                linear_acceleration: Vector3::zeros(),
                angular_velocity: Vector3::zeros(),
            },
        );

        assert_eq!(reading.sensor_id.raw(), 1);
        assert_eq!(reading.sensor_type, SensorType::Imu);
        assert_eq!(reading.timestamp, 0.001);
    }
}
