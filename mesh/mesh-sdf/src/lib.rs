//! Signed distance field computation for triangle meshes.
//!
//! This crate exposes two orthogonal oracles — [`UnsignedDistance`]
//! and [`Sign`] — and a composition struct [`Signed`] that combines
//! them into a signed-distance source. Concrete oracles:
//! [`TriMeshDistance`] (parry3d BVH-backed unsigned distance) and
//! [`PseudoNormalSign`] (parry's pseudo-normal inside test, fast but
//! fragile on cleaned scans).
//!
//! The previous monolithic `SignedDistanceField` is retained as a
//! deprecated type alias for `Signed<TriMeshDistance, PseudoNormalSign>`
//! so existing call sites keep building during consumer migration.
//! New consumers should compose the oracles explicitly — see
//! `docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md` for the architecture
//! rationale and the choice between `PseudoNormalSign` (well-formed
//! synthetic meshes) and `FloodFillSign` (cleaned body-part scans;
//! shipped in D.2).
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with zero Bevy dependencies.
//!
//! # Examples
//!
//! ## Explicit composition (fast, well-formed meshes)
//!
//! ```
//! use mesh_types::{IndexedMesh, Point3};
//! use mesh_sdf::{PseudoNormalSign, Signed, TriMeshDistance, UnsignedDistance};
//!
//! // Create a simple triangle mesh
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
//! mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
//! mesh.vertices.push(Point3::new(5.0, 10.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Build the distance oracle once; share its BVH with a sign oracle.
//! let distance = TriMeshDistance::new(mesh).unwrap();
//! let sign = PseudoNormalSign::from_distance(&distance);
//! let sdf = Signed { distance, sign };
//! let _signed = sdf.evaluate(Point3::new(5.0, 5.0, 5.0));
//! let _unsigned = sdf.unsigned_distance(Point3::new(5.0, 5.0, 5.0));
//! ```
//!
//! ## Recommended for cleaned scans (robust sign via flood fill)
//!
//! ```
//! use mesh_types::{Aabb, IndexedMesh, Point3};
//! use mesh_sdf::{WALL_THRESHOLD_FACTOR_DEFAULT, flood_filled_sdf};
//!
//! // Closed tetrahedron — stand-in for a cleaned scan.
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
//! mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
//! mesh.vertices.push(Point3::new(0.5, 0.866, 0.0));
//! mesh.vertices.push(Point3::new(0.5, 0.289, 0.816));
//! mesh.faces.push([0, 2, 1]);
//! mesh.faces.push([0, 1, 3]);
//! mesh.faces.push([1, 2, 3]);
//! mesh.faces.push([2, 0, 3]);
//!
//! // Bounds extend past the mesh on every axis so the corners can
//! // seed the outside flood.
//! let bounds = Aabb::new(
//!     Point3::new(-1.0, -1.0, -1.0),
//!     Point3::new(2.0, 2.0, 2.0),
//! );
//! let (sdf, _report) = flood_filled_sdf(
//!     mesh,
//!     bounds,
//!     0.1,
//!     WALL_THRESHOLD_FACTOR_DEFAULT,
//! ).unwrap();
//!
//! // `sdf` is a `Signed<TriMeshDistance, FloodFillSign>` and impls
//! // `cf_design::Sdf` via cf-design's blanket impl — drop-in for
//! // `Solid::from_sdf` and `Arc<dyn Sdf>` consumers.
//! assert!(sdf.evaluate(Point3::new(0.5, 0.3, 0.2)) < 0.0);
//! ```
//!
//! # Use Cases
//!
//! - **Mesh offsetting**: Compute offset surfaces by moving along the SDF gradient
//! - **Collision detection**: Quick inside/outside tests
//! - **Ray marching**: Efficient rendering of implicit surfaces
//! - **Boolean operations**: Combine meshes using SDF operations (union, intersection, difference)

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]

mod error;
mod flood_fill;
mod oracle;
mod sdf;
mod sdf_adapter;

#[cfg(test)]
mod test_fixtures;

pub use error::{SdfError, SdfResult};
pub use flood_fill::{
    CachedGridSdf, FloodFillSign, FloodFilledSdfBuildError, WALL_THRESHOLD_FACTOR_DEFAULT,
    flood_filled_sdf,
};
pub use oracle::{FloodFillError, FloodFillReport, Region, Sign, Signed, UnsignedDistance};
pub use sdf::{PseudoNormalSign, TriMeshDistance};
#[allow(deprecated)]
pub use sdf::{SignedDistanceField, signed_distance, unsigned_distance};
