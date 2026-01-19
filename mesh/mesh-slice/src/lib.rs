//! 3D printing slicing and layer preview for meshes.
//!
//! This crate provides tools for generating 2D slice previews of meshes,
//! calculating per-layer statistics, and estimating print times.
//!
//! # Features
//!
//! - **Slicing**: Generate layer-by-layer previews of meshes
//! - **Print time estimation**: Calculate estimated print times
//! - **Layer statistics**: Analyze per-layer area, perimeter, and island count
//! - **FDM validation**: Check mesh for FDM printing compatibility
//! - **SLA validation**: Check mesh for resin printing compatibility
//! - **SVG export**: Export layers as SVG for visualization
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
//! use mesh_slice::{slice_mesh, SliceParams, validate_for_fdm, FdmParams};
//!
//! // Create a unit cube
//! let cube = unit_cube();
//!
//! // Generate slices
//! let result = slice_mesh(&cube, &SliceParams::default());
//! println!("Total layers: {}", result.layer_count);
//! println!("Estimated print time: {:.1} minutes", result.estimated_print_time);
//!
//! // Validate for FDM printing
//! let fdm_result = validate_for_fdm(&cube, &FdmParams::default());
//! println!("{}", fdm_result.summary);
//! ```
//!
//! # Coordinate System
//!
//! Uses a **right-handed coordinate system**:
//! - X: width (left/right)
//! - Y: depth (front/back)
//! - Z: height (up/down, print direction)

#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

mod error;
mod export;
mod layer;
mod params;
mod result;
mod slicer;
mod validate;

// Re-export main types and functions
pub use error::{SliceError, SliceResult};
pub use export::{SvgExportParams, export_layer_svg};
pub use layer::{Contour, Layer, LayerBounds, LayerStats};
pub use params::SliceParams;
pub use result::SliceResult as SliceOutput;
pub use slicer::{slice_mesh, slice_preview};
pub use validate::{
    FdmParams, FdmValidationResult, GapIssue, SlaParams, SlaValidationResult, SmallFeatureIssue,
    ThinWallIssue, validate_for_fdm, validate_for_sla,
};

// Re-export nalgebra types for convenience
pub use nalgebra::{Point3, Vector3};
