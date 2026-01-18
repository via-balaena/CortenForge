//! Wall thickness analysis for 3D meshes.
//!
//! This crate provides tools for computing and analyzing wall thickness
//! in closed meshes. Wall thickness is crucial for 3D printing where
//! thin regions may fail structurally or print incorrectly.
//!
//! # Algorithm
//!
//! Wall thickness is computed by casting rays from each vertex inward
//! (opposite to the vertex normal) and finding the closest intersection
//! with the mesh surface. This gives the local wall thickness at each point.
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
//! use mesh_thickness::{analyze_thickness, ThicknessParams};
//!
//! let cube = unit_cube();
//! let result = analyze_thickness(&cube, &ThicknessParams::default());
//!
//! println!("Min thickness: {:.3}", result.min_thickness);
//! println!("Max thickness: {:.3}", result.max_thickness);
//! println!("Thin regions: {}", result.thin_regions.len());
//! ```
//!
//! # Use Cases
//!
//! - **3D Printing Validation**: Check if walls are thick enough for printing
//! - **Structural Analysis**: Find weak/thin areas in a model
//! - **Quality Control**: Verify manufactured parts meet specifications

#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

mod analysis;
mod error;
mod params;
mod result;

// Re-export main types and functions
pub use analysis::analyze_thickness;
pub use error::{ThicknessError, ThicknessResult};
pub use params::ThicknessParams;
pub use result::{AnalysisResult, ThinRegion};
