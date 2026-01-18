//! Geodesic (surface) distance computation for triangle meshes.
//!
//! This crate provides algorithms for computing distances along mesh surfaces:
//!
//! - **Dijkstra's algorithm** - Exact shortest path along mesh edges
//! - **Multi-source geodesics** - Distance from multiple seed points
//! - **Distance field** - Per-vertex distance values
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - CLI tools
//! - Web applications (WASM)
//! - Servers
//! - Other game engines
//! - Python bindings
//!
//! # Algorithm
//!
//! The implementation uses Dijkstra's algorithm on the mesh edge graph.
//! Edge weights are Euclidean distances between vertices.
//!
//! This gives exact geodesic distances **along edges**, which is an approximation
//! of the true geodesic distance (shortest path on the surface). For denser meshes,
//! this approximation is very accurate.
//!
//! # Example
//!
//! ```
//! use mesh_geodesic::{GeodesicSolver, DistanceField};
//! use mesh_types::{IndexedMesh, Vertex, MeshTopology};
//!
//! // Create a simple mesh (triangle)
//! let mut mesh = IndexedMesh::new();
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
//! mesh.faces.push([0, 1, 2]);
//!
//! // Compute distances from vertex 0
//! let solver = GeodesicSolver::new(&mesh);
//! let distances = solver.compute_from_vertex(0);
//!
//! assert!(distances.distance(0) < 1e-10); // Distance to self is 0
//! assert!((distances.distance(1) - 1.0).abs() < 1e-10); // Distance to v1
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - >=90% test coverage
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

mod adjacency;
mod dijkstra;
mod distance;
mod error;

pub use adjacency::AdjacencyList;
pub use dijkstra::GeodesicSolver;
pub use distance::DistanceField;
pub use error::{GeodesicError, GeodesicResult};

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::{IndexedMesh, Vertex};

    fn create_line_mesh() -> IndexedMesh {
        // Three vertices in a line: 0 -- 1 -- 2
        // Two triangles sharing edge 0-1 and 1-2
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(2.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 0.0)); // Top vertex
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh
    }

    #[test]
    fn basic_geodesic() {
        let mesh = create_line_mesh();
        let solver = GeodesicSolver::new(&mesh);
        let distances = solver.compute_from_vertex(0);

        // Distance from 0 to 0 should be 0
        assert!(distances.distance(0) < 1e-10);

        // Distance from 0 to 1 should be 1.0
        assert!((distances.distance(1) - 1.0).abs() < 1e-10);

        // Distance from 0 to 2 should be 2.0 (via vertex 1)
        assert!((distances.distance(2) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn multi_source_geodesic() {
        let mesh = create_line_mesh();
        let solver = GeodesicSolver::new(&mesh);
        let distances = solver.compute_from_vertices(&[0, 2]);

        // Distance from {0, 2} to 1 should be 1.0 (equidistant from both)
        assert!((distances.distance(1) - 1.0).abs() < 1e-10);

        // Distance to sources should be 0
        assert!(distances.distance(0) < 1e-10);
        assert!(distances.distance(2) < 1e-10);
    }

    #[test]
    fn single_triangle_mesh() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        let solver = GeodesicSolver::new(&mesh);
        let distances = solver.compute_from_vertex(0);

        assert_eq!(distances.len(), 3);
        assert!(distances.distance(0) < 1e-10);
    }

    #[test]
    fn empty_mesh() {
        let mesh = IndexedMesh::new();
        let solver = GeodesicSolver::new(&mesh);
        let distances = solver.compute_from_vertices(&[]);

        assert_eq!(distances.len(), 0);
    }
}
