//! Indexed triangle mesh.

use crate::{Aabb, MeshBounds, MeshTopology, Triangle, Vertex};
use nalgebra::Vector3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// An indexed triangle mesh.
///
/// This is the primary mesh type for CortenForge. It stores vertices
/// and faces separately, with faces referencing vertices by index.
///
/// # Memory Layout
///
/// - `vertices`: `Vec<Vertex>` - Vertex positions and attributes
/// - `faces`: `Vec<[u32; 3]>` - Triangle faces as vertex indices
///
/// # Winding Order
///
/// Faces use **counter-clockwise (CCW) winding** when viewed from outside.
/// This means normals point outward by the right-hand rule.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex, Point3, MeshTopology};
///
/// // Create a single triangle
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// assert_eq!(mesh.vertex_count(), 3);
/// assert_eq!(mesh.face_count(), 1);
/// ```
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct IndexedMesh {
    /// Vertex data.
    pub vertices: Vec<Vertex>,

    /// Triangle faces as indices into the vertex array.
    /// Each face is `[v0, v1, v2]` with counter-clockwise winding.
    pub faces: Vec<[u32; 3]>,
}

impl IndexedMesh {
    /// Create a new empty mesh.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, MeshTopology};
    ///
    /// let mesh = IndexedMesh::new();
    /// assert!(mesh.is_empty());
    /// ```
    #[inline]
    #[must_use]
    pub const fn new() -> Self {
        Self {
            vertices: Vec::new(),
            faces: Vec::new(),
        }
    }

    /// Create a mesh with pre-allocated capacity.
    ///
    /// # Arguments
    ///
    /// * `vertex_count` - Expected number of vertices
    /// * `face_count` - Expected number of faces
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, MeshTopology};
    ///
    /// let mesh = IndexedMesh::with_capacity(1000, 2000);
    /// assert!(mesh.is_empty());
    /// ```
    #[inline]
    #[must_use]
    pub fn with_capacity(vertex_count: usize, face_count: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(vertex_count),
            faces: Vec::with_capacity(face_count),
        }
    }

    /// Create a mesh from vertices and faces.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, Vertex, MeshTopology};
    ///
    /// let vertices = vec![
    ///     Vertex::from_coords(0.0, 0.0, 0.0),
    ///     Vertex::from_coords(1.0, 0.0, 0.0),
    ///     Vertex::from_coords(0.0, 1.0, 0.0),
    /// ];
    /// let faces = vec![[0, 1, 2]];
    ///
    /// let mesh = IndexedMesh::from_parts(vertices, faces);
    /// assert_eq!(mesh.face_count(), 1);
    /// ```
    #[inline]
    #[must_use]
    pub const fn from_parts(vertices: Vec<Vertex>, faces: Vec<[u32; 3]>) -> Self {
        Self { vertices, faces }
    }

    /// Create a mesh from raw coordinate and index data.
    ///
    /// This is a convenience method for creating meshes from arrays.
    ///
    /// # Arguments
    ///
    /// * `positions` - Flat array of vertex positions `[x0, y0, z0, x1, y1, z1, ...]`
    /// * `indices` - Flat array of face indices `[v0a, v1a, v2a, v0b, v1b, v2b, ...]`
    ///
    /// # Panics
    ///
    /// This function does not panic, but returns an empty mesh if:
    /// - `positions.len()` is not divisible by 3
    /// - `indices.len()` is not divisible by 3
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, MeshTopology};
    ///
    /// let positions = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    /// let indices = [0, 1, 2];
    ///
    /// let mesh = IndexedMesh::from_raw(&positions, &indices);
    /// assert_eq!(mesh.vertex_count(), 3);
    /// assert_eq!(mesh.face_count(), 1);
    /// ```
    #[must_use]
    pub fn from_raw(positions: &[f64], indices: &[u32]) -> Self {
        if positions.len() % 3 != 0 || indices.len() % 3 != 0 {
            return Self::new();
        }

        let vertices = positions
            .chunks_exact(3)
            .map(|c| Vertex::from_coords(c[0], c[1], c[2]))
            .collect();

        let faces = indices
            .chunks_exact(3)
            .map(|c| [c[0], c[1], c[2]])
            .collect();

        Self { vertices, faces }
    }

    /// Rotate mesh 90 degrees around X axis (Y becomes Z, Z becomes -Y).
    ///
    /// This is useful for converting between coordinate systems.
    pub fn rotate_x_90(&mut self) {
        for vertex in &mut self.vertices {
            let old_y = vertex.position.y;
            let old_z = vertex.position.z;
            vertex.position.y = -old_z;
            vertex.position.z = old_y;

            if let Some(ref mut normal) = vertex.attributes.normal {
                let prev_normal_y = normal.y;
                let prev_normal_z = normal.z;
                normal.y = -prev_normal_z;
                normal.z = prev_normal_y;
            }
        }
    }

    /// Translate mesh so minimum Z is at zero.
    ///
    /// Useful for placing meshes on a build plate.
    pub fn place_on_z_zero(&mut self) {
        let bounds = self.bounds();
        if !bounds.is_empty() {
            let offset = -bounds.min.z;
            for vertex in &mut self.vertices {
                vertex.position.z += offset;
            }
        }
    }

    /// Translate mesh by the given vector.
    pub fn translate(&mut self, offset: Vector3<f64>) {
        for vertex in &mut self.vertices {
            vertex.position += offset;
        }
    }

    /// Scale mesh uniformly around the origin.
    pub fn scale(&mut self, factor: f64) {
        for vertex in &mut self.vertices {
            vertex.position.coords *= factor;
        }
    }

    /// Scale mesh uniformly around its center.
    pub fn scale_centered(&mut self, factor: f64) {
        let center = self.bounds().center();
        for vertex in &mut self.vertices {
            vertex.position = center + (vertex.position - center) * factor;
        }
    }

    /// Compute the signed volume of the mesh.
    ///
    /// Uses the divergence theorem: the signed volume is the sum of signed
    /// tetrahedra volumes formed by each face and the origin.
    ///
    /// For a closed mesh with outward-facing normals (CCW winding when
    /// viewed from outside), this returns a positive value.
    ///
    /// # Returns
    ///
    /// - Positive value: normals point outward (correct orientation)
    /// - Negative value: normals point inward (inside-out mesh)
    /// - Near-zero: mesh is not closed or has inconsistent winding
    ///
    /// # Note
    ///
    /// This calculation assumes the mesh is closed (watertight). For open
    /// meshes, the result is not meaningful as a volume measurement.
    #[must_use]
    pub fn signed_volume(&self) -> f64 {
        let mut volume = 0.0;

        for &[i0, i1, i2] in &self.faces {
            let v0 = &self.vertices[i0 as usize].position;
            let v1 = &self.vertices[i1 as usize].position;
            let v2 = &self.vertices[i2 as usize].position;

            // Signed volume of tetrahedron with origin = (v0 · (v1 × v2)) / 6
            // Using mul_add for better numerical accuracy and performance
            let cross = Vector3::new(
                v1.y.mul_add(v2.z, -(v1.z * v2.y)),
                v1.z.mul_add(v2.x, -(v1.x * v2.z)),
                v1.x.mul_add(v2.y, -(v1.y * v2.x)),
            );
            volume += v0.z.mul_add(cross.z, v0.x.mul_add(cross.x, v0.y * cross.y));
        }

        volume / 6.0
    }

    /// Compute the absolute volume of the mesh.
    ///
    /// Returns the absolute value of `signed_volume()`.
    #[inline]
    #[must_use]
    pub fn volume(&self) -> f64 {
        self.signed_volume().abs()
    }

    /// Check if the mesh appears to be inside-out.
    ///
    /// A mesh is considered inside-out if its signed volume is negative.
    #[inline]
    #[must_use]
    pub fn is_inside_out(&self) -> bool {
        self.signed_volume() < 0.0
    }

    /// Compute the total surface area of the mesh.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        self.triangles().map(|tri| tri.area()).sum()
    }

    /// Flip all face normals by reversing winding order.
    pub fn flip_normals(&mut self) {
        for face in &mut self.faces {
            face.swap(1, 2);
        }
        // Also flip vertex normals if present
        for vertex in &mut self.vertices {
            if let Some(ref mut normal) = vertex.attributes.normal {
                *normal = -*normal;
            }
        }
    }

    /// Clear all vertex normals.
    pub fn clear_normals(&mut self) {
        for vertex in &mut self.vertices {
            vertex.attributes.normal = None;
        }
    }

    /// Reserve capacity for additional vertices and faces.
    pub fn reserve(&mut self, additional_vertices: usize, additional_faces: usize) {
        self.vertices.reserve(additional_vertices);
        self.faces.reserve(additional_faces);
    }

    /// Merge another mesh into this one.
    ///
    /// The other mesh's vertices and faces are appended, with face
    /// indices adjusted appropriately.
    ///
    /// # Note
    ///
    /// This function uses u32 vertex indices, which supports up to ~4 billion vertices.
    /// Meshes exceeding this limit are not supported.
    #[allow(clippy::cast_possible_truncation)]
    // Truncation: mesh indices are u32, so vertex counts > 4B are unsupported by design
    pub fn merge(&mut self, other: &Self) {
        let vertex_offset = self.vertices.len() as u32;

        self.vertices.extend(other.vertices.iter().cloned());

        for face in &other.faces {
            self.faces.push([
                face[0] + vertex_offset,
                face[1] + vertex_offset,
                face[2] + vertex_offset,
            ]);
        }
    }
}

impl MeshTopology for IndexedMesh {
    #[inline]
    fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    #[inline]
    fn face_count(&self) -> usize {
        self.faces.len()
    }

    fn vertex(&self, index: usize) -> Option<&Vertex> {
        self.vertices.get(index)
    }

    fn face(&self, index: usize) -> Option<[u32; 3]> {
        self.faces.get(index).copied()
    }

    fn triangle(&self, face_index: usize) -> Option<Triangle> {
        self.faces.get(face_index).map(|&[i0, i1, i2]| Triangle {
            v0: self.vertices[i0 as usize].position,
            v1: self.vertices[i1 as usize].position,
            v2: self.vertices[i2 as usize].position,
        })
    }

    fn vertices(&self) -> impl Iterator<Item = &Vertex> {
        self.vertices.iter()
    }

    fn faces(&self) -> impl Iterator<Item = [u32; 3]> {
        self.faces.iter().copied()
    }

    fn triangles(&self) -> impl Iterator<Item = Triangle> {
        self.faces.iter().map(|&[i0, i1, i2]| Triangle {
            v0: self.vertices[i0 as usize].position,
            v1: self.vertices[i1 as usize].position,
            v2: self.vertices[i2 as usize].position,
        })
    }
}

impl MeshBounds for IndexedMesh {
    fn bounds(&self) -> Aabb {
        if self.vertices.is_empty() {
            return Aabb::empty();
        }

        let positions = self.vertices.iter().map(|v| &v.position);
        Aabb::from_points(positions)
    }
}

/// Helper function to create a unit cube mesh.
///
/// Creates a cube from (0,0,0) to (1,1,1) with outward-facing normals.
///
/// # Example
///
/// ```
/// use mesh_types::{unit_cube, MeshTopology};
///
/// let cube = unit_cube();
/// assert_eq!(cube.vertex_count(), 8);
/// assert_eq!(cube.face_count(), 12);
/// ```
#[must_use]
pub fn unit_cube() -> IndexedMesh {
    let mut mesh = IndexedMesh::with_capacity(8, 12);

    // 8 vertices of the cube
    mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0)); // 0
    mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0)); // 1
    mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0)); // 2
    mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0)); // 3
    mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0)); // 4
    mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 1.0)); // 5
    mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 1.0)); // 6
    mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 1.0)); // 7

    // 12 triangles (2 per face), CCW winding when viewed from outside

    // Bottom face (z=0) - normal points -Z
    mesh.faces.push([0, 2, 1]);
    mesh.faces.push([0, 3, 2]);

    // Top face (z=1) - normal points +Z
    mesh.faces.push([4, 5, 6]);
    mesh.faces.push([4, 6, 7]);

    // Front face (y=0) - normal points -Y
    mesh.faces.push([0, 1, 5]);
    mesh.faces.push([0, 5, 4]);

    // Back face (y=1) - normal points +Y
    mesh.faces.push([3, 7, 6]);
    mesh.faces.push([3, 6, 2]);

    // Left face (x=0) - normal points -X
    mesh.faces.push([0, 4, 7]);
    mesh.faces.push([0, 7, 3]);

    // Right face (x=1) - normal points +X
    mesh.faces.push([1, 2, 6]);
    mesh.faces.push([1, 6, 5]);

    mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mesh_is_empty() {
        let mesh = IndexedMesh::new();
        assert!(mesh.is_empty());

        let mut mesh2 = IndexedMesh::new();
        mesh2.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        assert!(mesh2.is_empty()); // no faces

        mesh2.faces.push([0, 0, 0]);
        assert!(!mesh2.is_empty());
    }

    #[test]
    fn mesh_from_raw() {
        let positions = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let indices = [0, 1, 2];

        let mesh = IndexedMesh::from_raw(&positions, &indices);
        assert_eq!(mesh.vertex_count(), 3);
        assert_eq!(mesh.face_count(), 1);
    }

    #[test]
    fn mesh_bounds() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 5.0, 3.0));
        mesh.vertices.push(Vertex::from_coords(-2.0, 8.0, 1.0));

        let bounds = mesh.bounds();
        assert!((bounds.min.x - (-2.0)).abs() < f64::EPSILON);
        assert!((bounds.min.y - 0.0).abs() < f64::EPSILON);
        assert!((bounds.min.z - 0.0).abs() < f64::EPSILON);
        assert!((bounds.max.x - 10.0).abs() < f64::EPSILON);
        assert!((bounds.max.y - 8.0).abs() < f64::EPSILON);
        assert!((bounds.max.z - 3.0).abs() < f64::EPSILON);
    }

    #[test]
    fn empty_mesh_bounds() {
        let mesh = IndexedMesh::new();
        assert!(mesh.bounds().is_empty());
    }

    #[test]
    fn unit_cube_volume() {
        let cube = unit_cube();
        let vol = cube.signed_volume();
        assert!(
            (vol - 1.0).abs() < 1e-10,
            "Unit cube volume should be 1.0, got {vol}"
        );
    }

    #[test]
    fn unit_cube_surface_area() {
        let cube = unit_cube();
        let area = cube.surface_area();
        assert!(
            (area - 6.0).abs() < 1e-10,
            "Unit cube surface area should be 6.0, got {area}"
        );
    }

    #[test]
    fn unit_cube_not_inside_out() {
        let cube = unit_cube();
        assert!(!cube.is_inside_out());
    }

    #[test]
    fn flipped_cube_inside_out() {
        let mut cube = unit_cube();
        cube.flip_normals();
        assert!(cube.is_inside_out());
    }

    #[test]
    fn mesh_merge() {
        let mut mesh1 = IndexedMesh::new();
        mesh1.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh1.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh1.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh1.faces.push([0, 1, 2]);

        let mut mesh2 = IndexedMesh::new();
        mesh2.vertices.push(Vertex::from_coords(2.0, 0.0, 0.0));
        mesh2.vertices.push(Vertex::from_coords(3.0, 0.0, 0.0));
        mesh2.vertices.push(Vertex::from_coords(2.0, 1.0, 0.0));
        mesh2.faces.push([0, 1, 2]);

        mesh1.merge(&mesh2);
        assert_eq!(mesh1.vertex_count(), 6);
        assert_eq!(mesh1.face_count(), 2);
        // Second face should have offset indices
        assert_eq!(mesh1.faces[1], [3, 4, 5]);
    }

    #[test]
    fn mesh_translate() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));

        mesh.translate(Vector3::new(1.0, 2.0, 3.0));

        let pos = mesh.vertices[0].position;
        assert!((pos.x - 1.0).abs() < f64::EPSILON);
        assert!((pos.y - 2.0).abs() < f64::EPSILON);
        assert!((pos.z - 3.0).abs() < f64::EPSILON);
    }

    #[test]
    fn mesh_scale() {
        let mut cube = unit_cube();
        cube.scale(2.0);
        let vol = cube.volume();
        assert!((vol - 8.0).abs() < 1e-10);
    }
}
