//! Measurement and dimensioning tools for 3D meshes.
//!
//! This crate provides tools for measuring distances, extracting cross-sections,
//! and computing dimensions of meshes.
//!
//! # Features
//!
//! - **Dimensions**: Axis-aligned bounding box and derived measurements
//! - **Oriented Bounding Box**: Minimum-volume bounding box using PCA
//! - **Cross-Sections**: Plane-mesh intersection with area and perimeter
//! - **Distance**: Point-to-point and point-to-mesh distance calculations
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - CLI tools
//! - Web applications (WASM)
//! - Servers
//! - Other game engines
//!
//! # Example
//!
//! ```
//! use mesh_types::unit_cube;
//! use mesh_measure::{dimensions, cross_section, oriented_bounding_box};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create a unit cube
//! let cube = unit_cube();
//!
//! // Get dimensions
//! let dims = dimensions(&cube);
//! assert!((dims.width - 1.0).abs() < 1e-10);
//! assert!((dims.height - 1.0).abs() < 1e-10);
//!
//! // Get oriented bounding box
//! let obb = oriented_bounding_box(&cube);
//! assert!((obb.volume - 1.0).abs() < 0.01);
//!
//! // Extract a cross-section
//! let section = cross_section(&cube, Point3::new(0.0, 0.0, 0.5), Vector3::z());
//! assert!((section.area - 1.0).abs() < 0.1);
//! ```
//!
//! # Coordinate System
//!
//! Uses a **right-handed coordinate system**:
//! - X: width (left/right)
//! - Y: depth (front/back)
//! - Z: height (up/down)

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]

mod cross_section;
mod dimensions;
mod distance;
mod error;
mod obb;

// Re-export main types and functions
pub use cross_section::{
    CrossSection, area_at_height, circumference_at_height, cross_section, cross_sections,
};
pub use dimensions::{Dimensions, dimensions};
pub use distance::{
    DistanceMeasurement, closest_point_on_mesh, distance_to_mesh, measure_distance,
};
pub use error::{MeasureError, MeasureResult};
pub use obb::{OrientedBoundingBox, oriented_bounding_box};

// Re-export nalgebra types for convenience
pub use nalgebra::{Point3, Vector3};
