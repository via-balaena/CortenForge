//! Sensor simulation for physics.
//!
//! This crate provides sensor simulation for rigid body physics, including:
//!
//! - [`Imu`] - Inertial measurement unit (accelerometer + gyroscope)
//! - [`ForceTorqueSensor`] - Measures forces and torques at a body or joint
//! - [`TouchSensor`] - Detects contact with other bodies
//!
//! # Design Philosophy
//!
//! Sensors are attached to bodies and read data from the simulation state.
//! They follow the `MuJoCo` sensor model where sensors produce readings based on
//! the current state of the simulation.
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless training loops
//! - Hardware control code
//! - Analysis tools
//!
//! # Example
//!
//! ```ignore
//! use sim_sensor::{Imu, ImuConfig, SensorReading};
//! use sim_types::BodyId;
//!
//! // Create an IMU attached to body 1
//! let imu = Imu::new(BodyId::new(1), ImuConfig::default());
//!
//! // Read sensor data (would normally come from a World)
//! // let reading = imu.read(&world_state);
//! ```

#![doc(html_root_url = "https://docs.rs/sim-sensor/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]
#![allow(
    clippy::missing_const_for_fn,
    clippy::suboptimal_flops,
    clippy::cast_precision_loss
)]

mod error;
mod force_torque;
mod imu;
mod touch;
mod types;

pub use error::SensorError;
pub use force_torque::{ForceTorqueReading, ForceTorqueSensor, ForceTorqueSensorConfig};
pub use imu::{Imu, ImuConfig, ImuReading};
pub use touch::{TouchReading, TouchSensor, TouchSensorConfig};
pub use types::{SensorData, SensorId, SensorReading, SensorType};

/// Result type for sensor operations.
pub type Result<T> = std::result::Result<T, SensorError>;
