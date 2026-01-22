//! Mesh printability analysis for 3D printing.
//!
//! This crate provides tools for validating mesh geometry for 3D printing,
//! detecting issues like thin walls, excessive overhangs, and non-manifold
//! geometry. It supports multiple printing technologies (FDM, SLA, SLS, MJF).
//!
//! # Features
//!
//! - **Print validation**: Check meshes against printer constraints
//! - **Issue detection**: Find thin walls, overhangs, and topology problems
//! - **Auto-orientation**: Find the optimal print orientation
//! - **Support estimation**: Estimate support material requirements
//! - **Multi-technology**: Support for FDM, SLA, SLS, and MJF
//!
//! # Example
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_printability::{validate_for_printing, PrinterConfig};
//!
//! // Create a simple triangle mesh
//! let mesh = IndexedMesh::from_parts(
//!     vec![
//!         Vertex::from_coords(0.0, 0.0, 0.0),
//!         Vertex::from_coords(10.0, 0.0, 0.0),
//!         Vertex::from_coords(5.0, 10.0, 0.0),
//!     ],
//!     vec![[0, 1, 2]],
//! );
//!
//! // Validate for FDM printing
//! let config = PrinterConfig::fdm_default();
//! let result = validate_for_printing(&mesh, &config).unwrap();
//!
//! // Check if printable and show summary
//! println!("Printable: {}", result.is_printable());
//! println!("Summary: {}", result.summary());
//! ```
//!
//! # Printer Technologies
//!
//! Different printing technologies have different constraints:
//!
//! - **FDM** (Fused Deposition Modeling): Layer-by-layer filament printing
//!   - Needs supports for overhangs > 45°
//!   - Minimum wall thickness ~1mm
//!   - Bridge span limited to ~10mm
//!
//! - **SLA** (Stereolithography): Resin curing with UV light
//!   - Higher detail resolution
//!   - Thinner walls possible (~0.4mm)
//!   - More conservative overhang angles
//!
//! - **SLS** (Selective Laser Sintering): Powder bed fusion
//!   - No supports needed (powder supports part)
//!   - Good for complex geometries
//!   - Minimum wall ~0.7mm
//!
//! - **MJF** (Multi Jet Fusion): HP powder printing
//!   - Similar to SLS (no supports)
//!   - High detail and speed
//!   - Minimum wall ~0.5mm

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]

mod config;
mod error;
mod issues;
mod orientation;
mod regions;
mod validation;

pub use config::{PrintTechnology, PrinterConfig};
pub use error::{PrintabilityError, PrintabilityResult};
pub use issues::{IssueSeverity, PrintIssue, PrintIssueType};
pub use orientation::{
    OrientationResult, apply_orientation, find_optimal_orientation, place_on_build_plate,
};
pub use regions::{OverhangRegion, SupportRegion, ThinWallRegion};
pub use validation::{PrintValidation, validate_for_printing};
