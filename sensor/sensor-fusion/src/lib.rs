//! Sensor fusion utilities for CortenForge.
//!
//! This crate provides tools for synchronizing and transforming sensor data:
//!
//! # Stream Synchronization
//!
//! - [`StreamBuffer`] - Time-ordered buffer for sensor readings
//! - [`SyncPolicy`] - Policy for synchronizing multiple streams
//! - [`StreamSynchronizer`] - Synchronizes readings from multiple sensors
//!
//! # Temporal Interpolation
//!
//! - [`Interpolator`] - Interpolates between sensor readings
//! - [`InterpolationMethod`] - Linear, nearest, or custom interpolation
//!
//! # Coordinate Transforms
//!
//! - [`Transform3D`] - Rigid body transform (rotation + translation)
//! - [`TransformChain`] - Chain of transforms for sensor calibration
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - Real-time sensor processing
//! - Offline data analysis
//! - Calibration tools
//!
//! # Example
//!
//! ```
//! use sensor_fusion::{StreamBuffer, Transform3D};
//! use glam::Vec3;
//!
//! // Create a buffer for timestamped readings
//! let mut buffer = StreamBuffer::new(100);
//! buffer.push(0.0, 1.0);
//! buffer.push(0.1, 2.0);
//!
//! // Create a transform
//! let transform = Transform3D::from_translation(Vec3::new(1.0, 0.0, 0.0));
//! let point = Vec3::ZERO;
//! let transformed = transform.apply_point(point);
//! assert!((transformed.x - 1.0).abs() < 1e-6);
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]

mod buffer;
mod error;
mod interpolation;
mod sync;
mod transform;

// Re-export buffer types
pub use buffer::StreamBuffer;

// Re-export synchronization types
pub use sync::{StreamSynchronizer, SyncPolicy, SyncResult};

// Re-export interpolation types
pub use interpolation::{InterpolationMethod, Interpolator, lerp, lerp_factor};

// Re-export transform types
pub use transform::{Transform3D, TransformChain};

// Re-export error types
pub use error::{FusionError, Result};

/// Prelude for convenient imports.
pub mod prelude {
    pub use super::{
        FusionError, InterpolationMethod, Interpolator, StreamBuffer, StreamSynchronizer,
        SyncPolicy, SyncResult, Transform3D, TransformChain, lerp, lerp_factor,
    };
}
