//! Traits for mesh types.

use crate::{Aabb, Triangle, Vertex};
use nalgebra::Point3;

/// Trait for types that provide mesh topology information.
///
/// This trait defines the minimal interface for a mesh structure,
/// allowing algorithms to work with different mesh representations.
pub trait MeshTopology {
    /// Get the number of vertices.
    fn vertex_count(&self) -> usize;

    /// Get the number of faces (triangles).
    fn face_count(&self) -> usize;

    /// Check if the mesh is empty.
    fn is_empty(&self) -> bool {
        self.vertex_count() == 0 || self.face_count() == 0
    }

    /// Get a vertex by index.
    ///
    /// Returns `None` if the index is out of bounds.
    fn vertex(&self, index: usize) -> Option<&Vertex>;

    /// Get a face by index.
    ///
    /// Returns `None` if the index is out of bounds.
    /// The returned array contains vertex indices.
    fn face(&self, index: usize) -> Option<[u32; 3]>;

    /// Get a triangle by face index with resolved vertex positions.
    ///
    /// Returns `None` if the face index is out of bounds.
    fn triangle(&self, face_index: usize) -> Option<Triangle>;

    /// Iterate over all vertices.
    fn vertices(&self) -> impl Iterator<Item = &Vertex>;

    /// Iterate over all faces as vertex index triples.
    fn faces(&self) -> impl Iterator<Item = [u32; 3]>;

    /// Iterate over all triangles with resolved vertex positions.
    fn triangles(&self) -> impl Iterator<Item = Triangle>;
}

/// Trait for types that can compute a bounding box.
pub trait MeshBounds {
    /// Compute the axis-aligned bounding box.
    ///
    /// Returns an empty AABB if the mesh has no vertices.
    fn bounds(&self) -> Aabb;

    /// Compute the bounding box, returning `None` if empty.
    fn bounds_opt(&self) -> Option<Aabb> {
        let b = self.bounds();
        if b.is_empty() { None } else { Some(b) }
    }

    /// Get the center of the bounding box.
    fn center(&self) -> Point3<f64> {
        self.bounds().center()
    }
}
