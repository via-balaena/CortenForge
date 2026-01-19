//! Generate triangle meshes from curves.
//!
//! This crate provides tools for creating triangle meshes from curve data,
//! including tubes, sweeps, and other curve-based geometry.
//!
//! # Features
//!
//! - **Tube generation**: Create cylindrical tubes around polyline curves
//! - **Variable radius**: Tubes with varying radius along the curve
//! - **Parallel transport**: Rotation-minimizing frames for consistent orientation
//! - **End caps**: Optional end caps for closed geometry
//!
//! # Quick Start
//!
//! ```
//! use mesh_from_curves::{tube_from_polyline, TubeConfig};
//! use nalgebra::Point3;
//!
//! // Define a curve
//! let points = vec![
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(0.0, 0.0, 10.0),
//! ];
//!
//! // Generate a tube
//! let config = TubeConfig::default()
//!     .with_radius(0.5)
//!     .with_segments(16);
//!
//! let mesh = tube_from_polyline(&points, &config).unwrap();
//! assert!(!mesh.faces.is_empty());
//! ```
//!
//! # Variable Radius Tubes
//!
//! For tubes that vary in radius along their length:
//!
//! ```
//! use mesh_from_curves::tube_variable_radius;
//! use nalgebra::Point3;
//!
//! let points = vec![
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(0.0, 0.0, 5.0),
//!     Point3::new(0.0, 0.0, 10.0),
//! ];
//!
//! // Radius varies from 1.0 to 2.0 and back to 1.0
//! let radii = vec![1.0, 2.0, 1.0];
//!
//! let mesh = tube_variable_radius(&points, &radii, 16, true).unwrap();
//! ```

mod error;
mod frame;
mod tube;

pub use error::{CurveError, CurveResult};
pub use frame::{Frame, parallel_transport_frames};
pub use tube::{TubeConfig, tube_from_polyline, tube_variable_radius};
