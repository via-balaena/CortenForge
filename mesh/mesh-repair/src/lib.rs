//! Mesh repair operations for fixing common mesh issues.
//!
//! This crate provides tools for:
//! - Mesh validation (manifold checks, watertight checks)
//! - Vertex welding (merge nearby vertices)
//! - Degenerate triangle removal
//! - Duplicate face removal
//! - Unreferenced vertex removal
//! - Hole detection and filling
//! - Winding order correction
//! - Connected component analysis
//! - Self-intersection detection
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with zero Bevy dependencies.
//!
//! # Example
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_repair::{validate_mesh, repair_mesh, RepairParams};
//!
//! // Create a simple mesh
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Validate the mesh
//! let report = validate_mesh(&mesh);
//! println!("Boundary edges: {}", report.boundary_edge_count);
//!
//! // Repair the mesh
//! let result = repair_mesh(&mut mesh, &RepairParams::default());
//! println!("Vertices welded: {}", result.vertices_welded);
//! ```

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]

mod adjacency;
pub mod components;
mod error;
pub mod holes;
pub mod intersect;
mod repair;
mod validate;
pub mod winding;

pub use adjacency::MeshAdjacency;
pub use error::{RepairError, RepairResult};
pub use repair::{
    RepairParams, RepairResult as RepairSummary, remove_degenerate_triangles,
    remove_degenerate_triangles_enhanced, remove_duplicate_faces, remove_unreferenced_vertices,
    repair_mesh, weld_vertices,
};
pub use validate::{MeshReport, ValidationOptions, validate_mesh, validate_mesh_with_options};

// Re-export commonly used items from submodules
pub use components::{
    ComponentAnalysis, find_connected_components, keep_largest_component, remove_small_components,
    split_into_components,
};
pub use holes::{BoundaryLoop, detect_holes, fill_holes};
pub use intersect::{
    IntersectionParams, SelfIntersectionResult, detect_self_intersections, has_self_intersections,
};
pub use winding::{count_inconsistent_faces, fix_winding_order};
