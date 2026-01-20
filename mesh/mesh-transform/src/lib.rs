//! Mesh transformation, alignment, and analysis.
//!
//! This crate provides tools for:
//! - Mesh transformations (translate, rotate, scale)
//! - PCA (Principal Component Analysis) for mesh orientation
//! - RANSAC plane fitting for surface detection
//! - Mesh alignment to coordinate axes
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with zero Bevy dependencies.
//!
//! # Example
//!
//! ```
//! use mesh_transform::{Transform3D, pca_axes};
//! use mesh_types::{IndexedMesh, Vertex};
//!
//! // Create a simple mesh
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Compute PCA axes
//! let pca = pca_axes(&mesh);
//! assert!(pca.is_some());
//!
//! // Apply a transformation
//! let transform = Transform3D::translation(1.0, 2.0, 3.0);
//! let transformed = transform.apply_to_mesh(&mesh);
//! ```

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]

mod error;
mod pca;
mod plane;
mod ransac;
mod transform;

pub use error::{TransformError, TransformResult};
pub use pca::{PcaResult, pca_axes, pca_axes_checked, pca_from_points};
pub use plane::Plane;
pub use ransac::{RansacConfig, RansacResult, ransac_plane, ransac_plane_from_points};
pub use transform::Transform3D;
