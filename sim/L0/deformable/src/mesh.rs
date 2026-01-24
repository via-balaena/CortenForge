//! Mesh topology types for deformable bodies.
//!
//! This module provides types for representing mesh topology:
//!
//! - [`Edge`] - An edge connecting two vertices
//! - [`Triangle`] - A triangle face (for cloth/shells)
//! - [`Tetrahedron`] - A tetrahedron (for volumetric soft bodies)
//! - [`DeformableMesh`] - Complete mesh with vertices and topology

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::error::{DeformableError, Result};
use crate::types::Vertex;

/// An edge connecting two vertices.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Edge {
    /// Index of the first vertex.
    pub v0: usize,
    /// Index of the second vertex.
    pub v1: usize,
}

impl Edge {
    /// Create a new edge.
    #[must_use]
    pub const fn new(v0: usize, v1: usize) -> Self {
        Self { v0, v1 }
    }

    /// Get the edge with vertices in canonical order (sorted).
    #[must_use]
    pub const fn canonical(self) -> Self {
        if self.v0 <= self.v1 {
            self
        } else {
            Self {
                v0: self.v1,
                v1: self.v0,
            }
        }
    }

    /// Check if this edge contains the given vertex index.
    #[must_use]
    pub const fn contains(&self, v: usize) -> bool {
        self.v0 == v || self.v1 == v
    }

    /// Get the other vertex in this edge.
    ///
    /// Returns `None` if `v` is not in this edge.
    #[must_use]
    pub const fn other(&self, v: usize) -> Option<usize> {
        if self.v0 == v {
            Some(self.v1)
        } else if self.v1 == v {
            Some(self.v0)
        } else {
            None
        }
    }
}

/// A triangle face (three vertices).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Triangle {
    /// Indices of the three vertices.
    pub vertices: [usize; 3],
}

impl Triangle {
    /// Create a new triangle.
    #[must_use]
    pub const fn new(v0: usize, v1: usize, v2: usize) -> Self {
        Self {
            vertices: [v0, v1, v2],
        }
    }

    /// Get the edges of this triangle.
    #[must_use]
    pub const fn edges(&self) -> [Edge; 3] {
        [
            Edge::new(self.vertices[0], self.vertices[1]),
            Edge::new(self.vertices[1], self.vertices[2]),
            Edge::new(self.vertices[2], self.vertices[0]),
        ]
    }

    /// Compute the area of this triangle given vertex positions.
    #[must_use]
    pub fn area(&self, positions: &[Point3<f64>]) -> f64 {
        let p0 = &positions[self.vertices[0]];
        let p1 = &positions[self.vertices[1]];
        let p2 = &positions[self.vertices[2]];

        let e1 = p1 - p0;
        let e2 = p2 - p0;

        0.5 * e1.cross(&e2).norm()
    }

    /// Compute the normal of this triangle given vertex positions.
    #[must_use]
    pub fn normal(&self, positions: &[Point3<f64>]) -> Vector3<f64> {
        let p0 = &positions[self.vertices[0]];
        let p1 = &positions[self.vertices[1]];
        let p2 = &positions[self.vertices[2]];

        let e1 = p1 - p0;
        let e2 = p2 - p0;

        let n = e1.cross(&e2);
        let len = n.norm();
        if len > 1e-10 {
            n / len
        } else {
            Vector3::z() // Degenerate triangle
        }
    }

    /// Check if this triangle contains the given vertex index.
    #[must_use]
    pub const fn contains(&self, v: usize) -> bool {
        self.vertices[0] == v || self.vertices[1] == v || self.vertices[2] == v
    }
}

/// A tetrahedron (four vertices, four triangular faces).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Tetrahedron {
    /// Indices of the four vertices.
    pub vertices: [usize; 4],
}

impl Tetrahedron {
    /// Create a new tetrahedron.
    #[must_use]
    pub const fn new(v0: usize, v1: usize, v2: usize, v3: usize) -> Self {
        Self {
            vertices: [v0, v1, v2, v3],
        }
    }

    /// Get the edges of this tetrahedron.
    #[must_use]
    pub const fn edges(&self) -> [Edge; 6] {
        [
            Edge::new(self.vertices[0], self.vertices[1]),
            Edge::new(self.vertices[0], self.vertices[2]),
            Edge::new(self.vertices[0], self.vertices[3]),
            Edge::new(self.vertices[1], self.vertices[2]),
            Edge::new(self.vertices[1], self.vertices[3]),
            Edge::new(self.vertices[2], self.vertices[3]),
        ]
    }

    /// Get the triangular faces of this tetrahedron.
    #[must_use]
    pub const fn faces(&self) -> [Triangle; 4] {
        [
            Triangle::new(self.vertices[0], self.vertices[2], self.vertices[1]),
            Triangle::new(self.vertices[0], self.vertices[1], self.vertices[3]),
            Triangle::new(self.vertices[0], self.vertices[3], self.vertices[2]),
            Triangle::new(self.vertices[1], self.vertices[2], self.vertices[3]),
        ]
    }

    /// Compute the volume of this tetrahedron given vertex positions.
    ///
    /// Returns signed volume (negative if vertices are in wrong order).
    #[must_use]
    pub fn signed_volume(&self, positions: &[Point3<f64>]) -> f64 {
        let p0 = &positions[self.vertices[0]];
        let p1 = &positions[self.vertices[1]];
        let p2 = &positions[self.vertices[2]];
        let p3 = &positions[self.vertices[3]];

        let e1 = p1 - p0;
        let e2 = p2 - p0;
        let e3 = p3 - p0;

        e1.cross(&e2).dot(&e3) / 6.0
    }

    /// Compute the unsigned volume of this tetrahedron.
    #[must_use]
    pub fn volume(&self, positions: &[Point3<f64>]) -> f64 {
        self.signed_volume(positions).abs()
    }

    /// Check if this tetrahedron contains the given vertex index.
    #[must_use]
    pub const fn contains(&self, v: usize) -> bool {
        self.vertices[0] == v
            || self.vertices[1] == v
            || self.vertices[2] == v
            || self.vertices[3] == v
    }
}

/// A deformable mesh containing vertices and topology.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DeformableMesh {
    /// The vertices (particles) of the mesh.
    pub vertices: Vec<Vertex>,
    /// The edges of the mesh.
    pub edges: Vec<Edge>,
    /// The triangles of the mesh (for 2D/surface).
    pub triangles: Vec<Triangle>,
    /// The tetrahedra of the mesh (for 3D/volumetric).
    pub tetrahedra: Vec<Tetrahedron>,
    /// Adjacency list: for each vertex, the indices of connected vertices.
    adjacency: Vec<SmallVec<[usize; 8]>>,
}

impl Default for DeformableMesh {
    fn default() -> Self {
        Self::new()
    }
}

impl DeformableMesh {
    /// Create an empty mesh.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            vertices: Vec::new(),
            edges: Vec::new(),
            triangles: Vec::new(),
            tetrahedra: Vec::new(),
            adjacency: Vec::new(),
        }
    }

    /// Create a mesh with the given number of vertices.
    #[must_use]
    pub fn with_capacity(num_vertices: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(num_vertices),
            edges: Vec::new(),
            triangles: Vec::new(),
            tetrahedra: Vec::new(),
            adjacency: vec![SmallVec::new(); num_vertices],
        }
    }

    /// Get the number of vertices.
    #[must_use]
    pub fn num_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// Get the number of edges.
    #[must_use]
    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }

    /// Get the number of triangles.
    #[must_use]
    pub fn num_triangles(&self) -> usize {
        self.triangles.len()
    }

    /// Get the number of tetrahedra.
    #[must_use]
    pub fn num_tetrahedra(&self) -> usize {
        self.tetrahedra.len()
    }

    /// Add a vertex to the mesh.
    pub fn add_vertex(&mut self, vertex: Vertex) -> usize {
        let idx = self.vertices.len();
        self.vertices.push(vertex);
        self.adjacency.push(SmallVec::new());
        idx
    }

    /// Add an edge to the mesh.
    ///
    /// # Errors
    ///
    /// Returns an error if the vertex indices are out of bounds.
    pub fn add_edge(&mut self, edge: Edge) -> Result<usize> {
        if edge.v0 >= self.vertices.len() || edge.v1 >= self.vertices.len() {
            return Err(DeformableError::index_out_of_bounds(format!(
                "Edge vertices ({}, {}) out of bounds for {} vertices",
                edge.v0,
                edge.v1,
                self.vertices.len()
            )));
        }

        let idx = self.edges.len();
        self.edges.push(edge);

        // Update adjacency
        if !self.adjacency[edge.v0].contains(&edge.v1) {
            self.adjacency[edge.v0].push(edge.v1);
        }
        if !self.adjacency[edge.v1].contains(&edge.v0) {
            self.adjacency[edge.v1].push(edge.v0);
        }

        Ok(idx)
    }

    /// Add a triangle to the mesh.
    ///
    /// Also adds the edges if they don't exist.
    ///
    /// # Errors
    ///
    /// Returns an error if the vertex indices are out of bounds.
    pub fn add_triangle(&mut self, triangle: Triangle) -> Result<usize> {
        for &v in &triangle.vertices {
            if v >= self.vertices.len() {
                return Err(DeformableError::index_out_of_bounds(format!(
                    "Triangle vertex {} out of bounds for {} vertices",
                    v,
                    self.vertices.len()
                )));
            }
        }

        // Add edges if they don't exist
        // Safety: vertices are validated above, so add_edge cannot fail
        for edge in triangle.edges() {
            let canonical = edge.canonical();
            if !self.edges.iter().any(|e| e.canonical() == canonical) {
                self.add_edge(edge)?;
            }
        }

        let idx = self.triangles.len();
        self.triangles.push(triangle);
        Ok(idx)
    }

    /// Add a tetrahedron to the mesh.
    ///
    /// Also adds the edges and triangles if they don't exist.
    ///
    /// # Errors
    ///
    /// Returns an error if the vertex indices are out of bounds.
    pub fn add_tetrahedron(&mut self, tet: Tetrahedron) -> Result<usize> {
        for &v in &tet.vertices {
            if v >= self.vertices.len() {
                return Err(DeformableError::index_out_of_bounds(format!(
                    "Tetrahedron vertex {} out of bounds for {} vertices",
                    v,
                    self.vertices.len()
                )));
            }
        }

        // Add edges if they don't exist
        // Safety: vertices are validated above, so add_edge cannot fail
        for edge in tet.edges() {
            let canonical = edge.canonical();
            if !self.edges.iter().any(|e| e.canonical() == canonical) {
                self.add_edge(edge)?;
            }
        }

        let idx = self.tetrahedra.len();
        self.tetrahedra.push(tet);
        Ok(idx)
    }

    /// Get the neighbors of a vertex.
    #[must_use]
    pub fn neighbors(&self, vertex: usize) -> &[usize] {
        if vertex < self.adjacency.len() {
            &self.adjacency[vertex]
        } else {
            &[]
        }
    }

    /// Get the positions of all vertices.
    #[must_use]
    pub fn positions(&self) -> Vec<Point3<f64>> {
        self.vertices.iter().map(|v| v.position).collect()
    }

    /// Compute the bounding box of the mesh.
    #[must_use]
    pub fn bounding_box(&self) -> (Point3<f64>, Point3<f64>) {
        if self.vertices.is_empty() {
            return (Point3::origin(), Point3::origin());
        }

        let mut min = self.vertices[0].position;
        let mut max = self.vertices[0].position;

        for v in &self.vertices {
            min.x = min.x.min(v.position.x);
            min.y = min.y.min(v.position.y);
            min.z = min.z.min(v.position.z);
            max.x = max.x.max(v.position.x);
            max.y = max.y.max(v.position.y);
            max.z = max.z.max(v.position.z);
        }

        (min, max)
    }

    /// Compute the total mass of the mesh.
    #[must_use]
    pub fn total_mass(&self) -> f64 {
        self.vertices.iter().map(|v| v.mass).sum()
    }

    /// Compute the center of mass.
    #[must_use]
    pub fn center_of_mass(&self) -> Point3<f64> {
        if self.vertices.is_empty() {
            return Point3::origin();
        }

        let total_mass = self.total_mass();
        if total_mass <= 0.0 {
            // Average position if no mass
            let sum: Vector3<f64> = self.vertices.iter().map(|v| v.position.coords).sum();
            return Point3::from(sum / self.vertices.len() as f64);
        }

        let weighted_sum: Vector3<f64> = self
            .vertices
            .iter()
            .map(|v| v.position.coords * v.mass)
            .sum();

        Point3::from(weighted_sum / total_mass)
    }

    /// Compute the total surface area (sum of triangle areas).
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        let positions = self.positions();
        self.triangles.iter().map(|t| t.area(&positions)).sum()
    }

    /// Compute the total volume (sum of tetrahedron volumes).
    #[must_use]
    pub fn volume(&self) -> f64 {
        let positions = self.positions();
        self.tetrahedra
            .iter()
            .map(|t| t.signed_volume(&positions))
            .sum::<f64>()
            .abs()
    }

    /// Mark boundary vertices (vertices that are on triangles but not fully enclosed).
    pub fn compute_boundary_vertices(&mut self) {
        use hashbrown::HashMap;

        // Count how many triangles each edge belongs to
        let mut edge_count: HashMap<(usize, usize), usize> = HashMap::new();

        for tri in &self.triangles {
            for edge in tri.edges() {
                let key = if edge.v0 < edge.v1 {
                    (edge.v0, edge.v1)
                } else {
                    (edge.v1, edge.v0)
                };
                *edge_count.entry(key).or_insert(0) += 1;
            }
        }

        // Boundary edges are those with count == 1
        for ((v0, v1), count) in edge_count {
            if count == 1 {
                self.vertices[v0]
                    .flags
                    .insert(crate::types::VertexFlags::BOUNDARY);
                self.vertices[v1]
                    .flags
                    .insert(crate::types::VertexFlags::BOUNDARY);
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn test_edge() {
        let e = Edge::new(0, 1);
        assert_eq!(e.v0, 0);
        assert_eq!(e.v1, 1);
        assert!(e.contains(0));
        assert!(e.contains(1));
        assert!(!e.contains(2));
        assert_eq!(e.other(0), Some(1));
        assert_eq!(e.other(1), Some(0));
        assert_eq!(e.other(2), None);
    }

    #[test]
    fn test_edge_canonical() {
        let e1 = Edge::new(5, 3);
        let e2 = Edge::new(3, 5);

        assert_eq!(e1.canonical(), e2.canonical());
        assert_eq!(e1.canonical().v0, 3);
        assert_eq!(e1.canonical().v1, 5);
    }

    #[test]
    fn test_triangle_area() {
        let tri = Triangle::new(0, 1, 2);
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];

        let area = tri.area(&positions);
        assert!((area - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_triangle_normal() {
        let tri = Triangle::new(0, 1, 2);
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];

        let normal = tri.normal(&positions);
        assert!((normal.z - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_tetrahedron_volume() {
        let tet = Tetrahedron::new(0, 1, 2, 3);
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];

        let volume = tet.volume(&positions);
        // Volume of unit tetrahedron = 1/6
        assert!((volume - 1.0 / 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_mesh_basic() {
        let mut mesh = DeformableMesh::new();

        // Add vertices
        mesh.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1.0));
        mesh.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1.0));
        mesh.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), 1.0));

        assert_eq!(mesh.num_vertices(), 3);

        // Add edge
        mesh.add_edge(Edge::new(0, 1)).unwrap();
        assert_eq!(mesh.num_edges(), 1);

        // Check adjacency
        assert!(mesh.neighbors(0).contains(&1));
        assert!(mesh.neighbors(1).contains(&0));
    }

    #[test]
    fn test_mesh_triangle() {
        let mut mesh = DeformableMesh::new();

        mesh.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1.0));
        mesh.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1.0));
        mesh.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), 1.0));

        mesh.add_triangle(Triangle::new(0, 1, 2)).unwrap();

        assert_eq!(mesh.num_triangles(), 1);
        assert_eq!(mesh.num_edges(), 3); // Auto-added

        let area = mesh.surface_area();
        assert!((area - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_mesh_bounding_box() {
        let mut mesh = DeformableMesh::new();

        mesh.add_vertex(Vertex::new(Point3::new(-1.0, -2.0, -3.0), 1.0));
        mesh.add_vertex(Vertex::new(Point3::new(4.0, 5.0, 6.0), 1.0));

        let (min, max) = mesh.bounding_box();

        assert_eq!(min.x, -1.0);
        assert_eq!(min.y, -2.0);
        assert_eq!(min.z, -3.0);
        assert_eq!(max.x, 4.0);
        assert_eq!(max.y, 5.0);
        assert_eq!(max.z, 6.0);
    }

    #[test]
    fn test_mesh_center_of_mass() {
        let mut mesh = DeformableMesh::new();

        mesh.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1.0));
        mesh.add_vertex(Vertex::new(Point3::new(2.0, 0.0, 0.0), 1.0));

        let com = mesh.center_of_mass();
        assert!((com.x - 1.0).abs() < 1e-10);
    }
}
