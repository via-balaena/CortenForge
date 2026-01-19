//! Sensor bundle for multi-sensor data.
//!
//! Provides a container for synchronized readings from multiple sensors.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::{
    CameraFrame, DepthMap, EncoderReading, ForceTorqueReading, GpsReading, ImuReading, LidarScan,
    Timestamp,
};

/// A bundle of sensor readings from multiple sensors.
///
/// Used to group synchronized sensor data for processing in `sensor-fusion`
/// or for dataset storage. All sensors in a bundle share a common reference
/// timestamp.
///
/// # Example
///
/// ```
/// use sensor_types::{SensorBundle, ImuReading, CoordinateFrame, Timestamp};
///
/// let mut bundle = SensorBundle::new(Timestamp::from_secs_f64(1.0));
/// bundle.imu = Some(ImuReading::new(
///     Timestamp::from_secs_f64(1.0),
///     [0.0, 0.0, 9.81],
///     [0.0, 0.0, 0.0],
///     CoordinateFrame::Body,
/// ));
///
/// assert!(bundle.has_imu());
/// assert!(!bundle.is_empty());
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SensorBundle {
    /// Reference timestamp for the bundle.
    ///
    /// Individual sensor readings may have slightly different timestamps
    /// due to synchronization jitter.
    pub timestamp: Timestamp,

    /// IMU reading (acceleration, angular velocity).
    pub imu: Option<ImuReading>,

    /// Camera frames (may have multiple cameras).
    pub cameras: Vec<CameraFrame>,

    /// Depth map (from depth camera).
    pub depth: Option<DepthMap>,

    /// `LiDAR` scan (point cloud).
    pub lidar: Option<LidarScan>,

    /// Force/torque sensor readings (may have multiple sensors).
    pub force_torque: Vec<ForceTorqueReading>,

    /// Encoder readings (joint states).
    pub encoders: Option<EncoderReading>,

    /// GPS reading.
    pub gps: Option<GpsReading>,
}

impl SensorBundle {
    /// Creates a new empty sensor bundle.
    #[must_use]
    pub const fn new(timestamp: Timestamp) -> Self {
        Self {
            timestamp,
            imu: None,
            cameras: Vec::new(),
            depth: None,
            lidar: None,
            force_torque: Vec::new(),
            encoders: None,
            gps: None,
        }
    }

    /// Checks if the bundle is empty (no sensor data).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.imu.is_none()
            && self.cameras.is_empty()
            && self.depth.is_none()
            && self.lidar.is_none()
            && self.force_torque.is_empty()
            && self.encoders.is_none()
            && self.gps.is_none()
    }

    /// Returns the number of sensors with data in this bundle.
    #[must_use]
    pub fn sensor_count(&self) -> usize {
        let mut count = 0;
        if self.imu.is_some() {
            count += 1;
        }
        count += self.cameras.len();
        if self.depth.is_some() {
            count += 1;
        }
        if self.lidar.is_some() {
            count += 1;
        }
        count += self.force_torque.len();
        if self.encoders.is_some() {
            count += 1;
        }
        if self.gps.is_some() {
            count += 1;
        }
        count
    }

    /// Checks if IMU data is present.
    #[must_use]
    pub const fn has_imu(&self) -> bool {
        self.imu.is_some()
    }

    /// Checks if any camera data is present.
    #[must_use]
    pub fn has_camera(&self) -> bool {
        !self.cameras.is_empty()
    }

    /// Checks if depth data is present.
    #[must_use]
    pub const fn has_depth(&self) -> bool {
        self.depth.is_some()
    }

    /// Checks if `LiDAR` data is present.
    #[must_use]
    pub const fn has_lidar(&self) -> bool {
        self.lidar.is_some()
    }

    /// Checks if any force/torque data is present.
    #[must_use]
    pub fn has_force_torque(&self) -> bool {
        !self.force_torque.is_empty()
    }

    /// Checks if encoder data is present.
    #[must_use]
    pub const fn has_encoders(&self) -> bool {
        self.encoders.is_some()
    }

    /// Checks if GPS data is present.
    #[must_use]
    pub const fn has_gps(&self) -> bool {
        self.gps.is_some()
    }

    /// Returns the number of cameras in the bundle.
    #[must_use]
    pub fn camera_count(&self) -> usize {
        self.cameras.len()
    }

    /// Gets a camera frame by index.
    #[must_use]
    pub fn camera(&self, index: usize) -> Option<&CameraFrame> {
        self.cameras.get(index)
    }

    /// Returns the number of force/torque sensors in the bundle.
    #[must_use]
    pub fn force_torque_count(&self) -> usize {
        self.force_torque.len()
    }

    /// Gets a force/torque reading by index.
    #[must_use]
    pub fn force_torque_sensor(&self, index: usize) -> Option<&ForceTorqueReading> {
        self.force_torque.get(index)
    }

    /// Computes the maximum timestamp jitter relative to the bundle timestamp.
    ///
    /// Returns the maximum difference between any sensor's timestamp and
    /// the bundle's reference timestamp.
    #[must_use]
    pub fn max_timestamp_jitter(&self) -> crate::Duration {
        let mut max_jitter = crate::Duration::zero();

        if let Some(ref imu) = self.imu {
            let jitter = self.timestamp.abs_diff(imu.timestamp);
            if jitter > max_jitter {
                max_jitter = jitter;
            }
        }

        for camera in &self.cameras {
            let jitter = self.timestamp.abs_diff(camera.timestamp);
            if jitter > max_jitter {
                max_jitter = jitter;
            }
        }

        if let Some(ref depth) = self.depth {
            let jitter = self.timestamp.abs_diff(depth.timestamp);
            if jitter > max_jitter {
                max_jitter = jitter;
            }
        }

        if let Some(ref lidar) = self.lidar {
            let jitter = self.timestamp.abs_diff(lidar.timestamp);
            if jitter > max_jitter {
                max_jitter = jitter;
            }
        }

        for ft in &self.force_torque {
            let jitter = self.timestamp.abs_diff(ft.timestamp);
            if jitter > max_jitter {
                max_jitter = jitter;
            }
        }

        if let Some(ref encoders) = self.encoders {
            let jitter = self.timestamp.abs_diff(encoders.timestamp);
            if jitter > max_jitter {
                max_jitter = jitter;
            }
        }

        if let Some(ref gps) = self.gps {
            let jitter = self.timestamp.abs_diff(gps.timestamp);
            if jitter > max_jitter {
                max_jitter = jitter;
            }
        }

        max_jitter
    }

    /// Returns a list of sensor types present in this bundle.
    #[must_use]
    pub fn present_sensors(&self) -> Vec<SensorType> {
        let mut sensors = Vec::new();

        if self.imu.is_some() {
            sensors.push(SensorType::Imu);
        }
        for _ in 0..self.cameras.len() {
            sensors.push(SensorType::Camera);
        }
        if self.depth.is_some() {
            sensors.push(SensorType::Depth);
        }
        if self.lidar.is_some() {
            sensors.push(SensorType::Lidar);
        }
        for _ in 0..self.force_torque.len() {
            sensors.push(SensorType::ForceTorque);
        }
        if self.encoders.is_some() {
            sensors.push(SensorType::Encoder);
        }
        if self.gps.is_some() {
            sensors.push(SensorType::Gps);
        }

        sensors
    }
}

impl Default for SensorBundle {
    fn default() -> Self {
        Self::new(Timestamp::zero())
    }
}

/// Type of sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum SensorType {
    /// Inertial Measurement Unit.
    Imu,
    /// Camera (RGB or mono).
    Camera,
    /// Depth sensor.
    Depth,
    /// `LiDAR`.
    Lidar,
    /// Force/torque sensor.
    ForceTorque,
    /// Joint encoder.
    Encoder,
    /// GPS receiver.
    Gps,
}

impl SensorType {
    /// Returns a human-readable name for the sensor type.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Imu => "IMU",
            Self::Camera => "Camera",
            Self::Depth => "Depth",
            Self::Lidar => "LiDAR",
            Self::ForceTorque => "Force/Torque",
            Self::Encoder => "Encoder",
            Self::Gps => "GPS",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CoordinateFrame;

    #[test]
    fn bundle_empty() {
        let bundle = SensorBundle::new(Timestamp::zero());
        assert!(bundle.is_empty());
        assert_eq!(bundle.sensor_count(), 0);
    }

    #[test]
    fn bundle_with_imu() {
        let mut bundle = SensorBundle::new(Timestamp::from_secs_f64(1.0));
        bundle.imu = Some(ImuReading::zero(Timestamp::from_secs_f64(1.0)));

        assert!(!bundle.is_empty());
        assert!(bundle.has_imu());
        assert_eq!(bundle.sensor_count(), 1);
    }

    #[test]
    fn bundle_with_multiple_cameras() {
        let mut bundle = SensorBundle::new(Timestamp::from_secs_f64(1.0));

        bundle.cameras.push(CameraFrame {
            timestamp: Timestamp::from_secs_f64(1.0),
            image: vec![0u8; 640 * 480 * 4],
            width: 640,
            height: 480,
            encoding: crate::ImageEncoding::Rgba8,
            intrinsics: crate::CameraIntrinsics::default(),
            extrinsics: crate::CameraExtrinsics::default(),
        });

        bundle.cameras.push(CameraFrame {
            timestamp: Timestamp::from_secs_f64(1.0),
            image: vec![0u8; 640 * 480 * 4],
            width: 640,
            height: 480,
            encoding: crate::ImageEncoding::Rgba8,
            intrinsics: crate::CameraIntrinsics::default(),
            extrinsics: crate::CameraExtrinsics::default(),
        });

        assert!(bundle.has_camera());
        assert_eq!(bundle.camera_count(), 2);
        assert_eq!(bundle.sensor_count(), 2);
    }

    #[test]
    fn bundle_timestamp_jitter() {
        let mut bundle = SensorBundle::new(Timestamp::from_nanos(1_000_000_000));

        // IMU with 1ms jitter
        bundle.imu = Some(ImuReading::new(
            Timestamp::from_nanos(1_001_000_000),
            [0.0, 0.0, 9.81],
            [0.0, 0.0, 0.0],
            CoordinateFrame::Body,
        ));

        let jitter = bundle.max_timestamp_jitter();
        assert_eq!(jitter.as_millis(), 1);
    }

    #[test]
    fn bundle_present_sensors() {
        let mut bundle = SensorBundle::new(Timestamp::zero());
        bundle.imu = Some(ImuReading::zero(Timestamp::zero()));
        bundle.gps = Some(GpsReading::default());

        let sensors = bundle.present_sensors();
        assert_eq!(sensors.len(), 2);
        assert!(sensors.contains(&SensorType::Imu));
        assert!(sensors.contains(&SensorType::Gps));
    }

    #[test]
    fn sensor_type_names() {
        assert_eq!(SensorType::Imu.name(), "IMU");
        assert_eq!(SensorType::Camera.name(), "Camera");
        assert_eq!(SensorType::Lidar.name(), "LiDAR");
    }

    #[cfg(feature = "serde")]
    #[test]
    fn bundle_serialization() {
        let mut bundle = SensorBundle::new(Timestamp::from_secs_f64(1.0));
        bundle.imu = Some(ImuReading::zero(Timestamp::from_secs_f64(1.0)));
        bundle.gps = Some(GpsReading::new(
            Timestamp::from_secs_f64(1.0),
            37.7749,
            -122.4194,
        ));

        let json = serde_json::to_string(&bundle).ok();
        assert!(json.is_some());
    }
}
