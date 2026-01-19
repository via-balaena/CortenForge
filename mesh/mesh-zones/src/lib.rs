//! Zone assignment and region labeling for triangle meshes.
//!
//! This crate provides tools for assigning faces of a mesh to zones or regions.
//! Common use cases include:
//!
//! - Mesh segmentation based on surface geometry
//! - Region growing from seed faces
//! - Connected component detection
//! - Normal-based face grouping
//!
//! # Overview
//!
//! The crate is organized around the [`ZoneMap`] type, which stores
//! zone assignments for each face in a mesh. Zones are identified
//! by `u32` IDs, with 0 reserved for "unassigned".
//!
//! # Quick Start
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_zones::{segment_mesh, SegmentConfig, ZoneMap};
//!
//! // Create a simple mesh
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Automatically segment the mesh
//! let zone_map = segment_mesh(&mesh, &SegmentConfig::default()).unwrap();
//! println!("Found {} zones", zone_map.zone_count());
//! ```
//!
//! # Region Growing
//!
//! For more control, use region growing from seed faces:
//!
//! ```
//! use mesh_types::{IndexedMesh, Vertex};
//! use mesh_zones::{FaceAdjacency, ZoneMap, grow_region, GrowConfig};
//!
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! let adjacency = FaceAdjacency::from_mesh(&mesh);
//! let mut zone_map = ZoneMap::new(mesh.faces.len());
//!
//! // Grow from face 0 with flood fill
//! let count = grow_region(
//!     &mesh,
//!     &adjacency,
//!     &mut zone_map,
//!     &[0],
//!     1,
//!     &GrowConfig::flood_fill()
//! ).unwrap();
//!
//! println!("Assigned {} faces to zone 1", count);
//! ```

mod adjacency;
mod error;
mod grow;
mod segment;
mod zone_map;

pub use adjacency::FaceAdjacency;
pub use error::{ZoneError, ZoneResult};
pub use grow::{GrowConfig, flood_fill, grow_multiple_regions, grow_region};
pub use segment::{
    SegmentConfig, segment_by_components, segment_by_normal_direction, segment_mesh,
};
pub use zone_map::ZoneMap;
