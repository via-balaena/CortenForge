//! Hardware-agnostic sensor data types for CortenForge.
//!
//! This crate provides foundational types for raw sensor data used across:
//! - Real hardware drivers (`ROS2`, custom drivers)
//! - Bevy simulation plugins (simulated sensors)
//! - ML models (input data)
//! - Dataset storage (serialized sensor readings)
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - CLI tools
//! - Web applications (WASM)
//! - Servers
//! - Real hardware drivers
//! - Simulation environments
//!
//! # Sensor Types
//!
//! - [`ImuReading`] - Inertial measurement (acceleration, angular velocity)
//! - [`CameraFrame`] - Raw image with calibration data
//! - [`DepthMap`] - Per-pixel depth values
//! - [`LidarScan`] - 3D point cloud from `LiDAR`
//! - [`ForceTorqueReading`] - 6-axis force/torque sensor
//! - [`EncoderReading`] - Joint positions and velocities
//! - [`GpsReading`] - Global position with accuracy
//! - [`SensorBundle`] - Multi-sensor container for synchronized data
//!
//! # Coordinate Frames
//!
//! All sensor readings include a [`CoordinateFrame`] to specify the reference frame:
//! - `World` - Global world frame
//! - `Body` - Robot/vehicle body frame
//! - `Sensor(name)` - Sensor-specific frame
//!
//! # Time
//!
//! All readings use [`Timestamp`] for nanosecond-precision timing,
//! enabling precise temporal alignment in `sensor-fusion`.
//!
//! # Design Philosophy
//!
//! These are **raw sensor types**. Processing/inference types (`Frame`, `DetectionResult`)
//! belong in `ml-types`. This separation enables:
//! - Sim ↔ Real parity: same types for simulation and hardware
//! - Clean boundaries between sensing and perception
//!
//! # Example
//!
//! ```
//! use sensor_types::{ImuReading, CoordinateFrame, Timestamp};
//!
//! let imu = ImuReading {
//!     timestamp: Timestamp::from_secs_f64(1.0),
//!     acceleration: [0.0, 0.0, 9.81],
//!     angular_velocity: [0.0, 0.0, 0.1],
//!     frame: CoordinateFrame::Body,
//! };
//!
//! assert!(imu.acceleration[2] > 9.0);
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - ≥90% test coverage
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]

mod bundle;
mod camera;
mod depth;
mod encoder;
mod error;
mod force;
mod frame;
mod gps;
mod imu;
mod lidar;
mod time;

// Re-export core types
pub use bundle::{SensorBundle, SensorType};
pub use camera::{CameraExtrinsics, CameraFrame, CameraIntrinsics, ImageEncoding};
pub use depth::{DepthMap, DepthStats};
pub use encoder::EncoderReading;
pub use error::SensorError;
pub use force::ForceTorqueReading;
pub use frame::{CoordinateFrame, Pose3d, Transform3d};
pub use gps::{GpsAccuracy, GpsFixType, GpsReading};
pub use imu::ImuReading;
pub use lidar::{LidarPoint, LidarScan};
pub use time::{Duration, TimeRange, Timestamp};
