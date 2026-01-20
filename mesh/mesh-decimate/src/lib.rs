//! Mesh simplification using quadric error metrics.
//!
//! This crate provides mesh decimation (simplification) by iteratively collapsing edges
//! while minimizing geometric error using the Quadric Error Metrics (QEM) algorithm.
//!
//! # Features
//!
//! - **Edge collapse**: Iteratively collapse edges with lowest error
//! - **Quadric error metrics**: Minimize geometric error during simplification
//! - **Boundary preservation**: Optionally preserve mesh boundaries
//! - **Sharp feature preservation**: Optionally preserve sharp edges
//! - **Target control**: Specify target by triangle count or ratio
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
//! use mesh_decimate::{decimate_mesh, DecimateParams};
//!
//! // Create a mesh
//! let cube = unit_cube();
//!
//! // Decimate to 50% of original triangles
//! let result = decimate_mesh(&cube, &DecimateParams::with_target_ratio(0.5));
//! println!("{}", result);
//!
//! // Use aggressive settings for more reduction
//! let aggressive_result = decimate_mesh(&cube, &DecimateParams::aggressive());
//! println!("Aggressive: {}", aggressive_result);
//! ```
//!
//! # Algorithm
//!
//! The implementation uses the Quadric Error Metrics (QEM) algorithm:
//!
//! 1. For each vertex, compute a quadric matrix representing the sum of squared
//!    distances to the planes of adjacent faces
//! 2. For each edge, compute the optimal collapse position and error cost
//! 3. Iteratively collapse the edge with minimum cost until target is reached
//! 4. Handle boundary and sharp feature preservation as configured

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]

mod decimate;
mod error;
mod params;
mod quadric;
mod result;

// Re-export main types and functions
pub use decimate::decimate_mesh;
pub use error::{DecimateError, DecimateResult};
pub use params::DecimateParams;
pub use result::DecimationResult;
