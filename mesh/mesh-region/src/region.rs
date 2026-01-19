//! Mesh region definition.
//!
//! A [`MeshRegion`] represents a named subset of a mesh, defined by vertex
//! and/or face indices. Regions can be combined using set operations.

use hashbrown::{HashMap, HashSet};
use mesh_types::IndexedMesh;

use crate::selector::RegionSelector;

/// A named region of a mesh, defined by face and/or vertex indices.
///
/// Regions are the fundamental building block for selective operations on meshes,
/// such as variable thickness, material assignment, or localized processing.
///
/// # Example
///
/// ```
/// use mesh_region::MeshRegion;
///
/// // Create a region from vertex indices
/// let region = MeshRegion::from_vertices("heel_cup", [0, 1, 2, 3]);
/// assert_eq!(region.name(), "heel_cup");
/// assert_eq!(region.vertex_count(), 4);
/// ```
#[derive(Debug, Clone)]
pub struct MeshRegion {
    /// Unique name for this region.
    name: String,

    /// Vertex indices that belong to this region.
    vertices: HashSet<u32>,

    /// Face indices that belong to this region.
    faces: HashSet<u32>,

    /// Optional metadata for this region.
    metadata: HashMap<String, String>,

    /// Optional color for visualization (RGB, 0-255).
    color: Option<(u8, u8, u8)>,
}

impl MeshRegion {
    /// Create an empty region with a name.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::MeshRegion;
    ///
    /// let region = MeshRegion::new("my_region");
    /// assert!(region.is_empty());
    /// ```
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            vertices: HashSet::new(),
            faces: HashSet::new(),
            metadata: HashMap::new(),
            color: None,
        }
    }

    /// Create a region from vertex indices.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::MeshRegion;
    ///
    /// let region = MeshRegion::from_vertices("toe_box", [10, 11, 12]);
    /// assert_eq!(region.vertex_count(), 3);
    /// assert!(region.contains_vertex(10));
    /// ```
    #[must_use]
    pub fn from_vertices(name: impl Into<String>, vertices: impl IntoIterator<Item = u32>) -> Self {
        Self {
            name: name.into(),
            vertices: vertices.into_iter().collect(),
            faces: HashSet::new(),
            metadata: HashMap::new(),
            color: None,
        }
    }

    /// Create a region from face indices.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::MeshRegion;
    ///
    /// let region = MeshRegion::from_faces("bottom", [0, 1, 2, 3]);
    /// assert_eq!(region.face_count(), 4);
    /// assert!(region.contains_face(0));
    /// ```
    #[must_use]
    pub fn from_faces(name: impl Into<String>, faces: impl IntoIterator<Item = u32>) -> Self {
        Self {
            name: name.into(),
            vertices: HashSet::new(),
            faces: faces.into_iter().collect(),
            metadata: HashMap::new(),
            color: None,
        }
    }

    /// Create a region by applying a selector to a mesh.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, Vertex, Point3};
    /// use mesh_region::{MeshRegion, RegionSelector};
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
    /// mesh.vertices.push(Vertex::new(Point3::new(10.0, 0.0, 0.0)));
    /// mesh.vertices.push(Vertex::new(Point3::new(5.0, 10.0, 0.0)));
    /// mesh.faces.push([0, 1, 2]);
    ///
    /// let selector = RegionSelector::sphere(Point3::new(5.0, 5.0, 0.0), 10.0);
    /// let region = MeshRegion::from_selector(&mesh, "near_center", &selector);
    /// assert_eq!(region.vertex_count(), 3);
    /// ```
    #[must_use]
    pub fn from_selector(
        mesh: &IndexedMesh,
        name: impl Into<String>,
        selector: &RegionSelector,
    ) -> Self {
        let (vertices, faces) = selector.select(mesh);
        Self {
            name: name.into(),
            vertices,
            faces,
            metadata: HashMap::new(),
            color: None,
        }
    }

    /// Get the region name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Add a vertex to this region.
    pub fn add_vertex(&mut self, vertex_index: u32) {
        self.vertices.insert(vertex_index);
    }

    /// Add a face to this region.
    pub fn add_face(&mut self, face_index: u32) {
        self.faces.insert(face_index);
    }

    /// Add multiple vertices to this region.
    pub fn add_vertices(&mut self, indices: impl IntoIterator<Item = u32>) {
        self.vertices.extend(indices);
    }

    /// Add multiple faces to this region.
    pub fn add_faces(&mut self, indices: impl IntoIterator<Item = u32>) {
        self.faces.extend(indices);
    }

    /// Remove a vertex from this region.
    pub fn remove_vertex(&mut self, vertex_index: u32) -> bool {
        self.vertices.remove(&vertex_index)
    }

    /// Remove a face from this region.
    pub fn remove_face(&mut self, face_index: u32) -> bool {
        self.faces.remove(&face_index)
    }

    /// Check if a vertex is in this region.
    #[must_use]
    pub fn contains_vertex(&self, vertex_index: u32) -> bool {
        self.vertices.contains(&vertex_index)
    }

    /// Check if a face is in this region.
    #[must_use]
    pub fn contains_face(&self, face_index: u32) -> bool {
        self.faces.contains(&face_index)
    }

    /// Get the number of vertices in this region.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Get the number of faces in this region.
    #[must_use]
    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// Check if this region is empty (no vertices or faces).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty() && self.faces.is_empty()
    }

    /// Get an iterator over vertex indices.
    pub fn vertices(&self) -> impl Iterator<Item = u32> + '_ {
        self.vertices.iter().copied()
    }

    /// Get an iterator over face indices.
    pub fn faces(&self) -> impl Iterator<Item = u32> + '_ {
        self.faces.iter().copied()
    }

    /// Get a reference to the vertex index set.
    #[must_use]
    pub fn vertex_set(&self) -> &HashSet<u32> {
        &self.vertices
    }

    /// Get a reference to the face index set.
    #[must_use]
    pub fn face_set(&self) -> &HashSet<u32> {
        &self.faces
    }

    /// Expand the region to include all vertices of contained faces.
    ///
    /// After this operation, any face in the region will have all its
    /// vertices also in the region.
    pub fn expand_to_face_vertices(&mut self, mesh: &IndexedMesh) {
        for face_idx in &self.faces {
            if let Some(face) = mesh.faces.get(*face_idx as usize) {
                self.vertices.insert(face[0]);
                self.vertices.insert(face[1]);
                self.vertices.insert(face[2]);
            }
        }
    }

    /// Expand the region to include all faces that contain any region vertex.
    ///
    /// After this operation, any vertex in the region will have all its
    /// adjacent faces also in the region.
    pub fn expand_to_vertex_faces(&mut self, mesh: &IndexedMesh) {
        for (face_idx, face) in mesh.faces.iter().enumerate() {
            if self.vertices.contains(&face[0])
                || self.vertices.contains(&face[1])
                || self.vertices.contains(&face[2])
            {
                self.faces
                    .insert(u32::try_from(face_idx).unwrap_or(u32::MAX));
            }
        }
    }

    /// Set metadata for this region.
    pub fn set_metadata(&mut self, key: impl Into<String>, value: impl Into<String>) {
        self.metadata.insert(key.into(), value.into());
    }

    /// Get metadata for this region.
    #[must_use]
    pub fn get_metadata(&self, key: &str) -> Option<&str> {
        self.metadata.get(key).map(String::as_str)
    }

    /// Get all metadata.
    #[must_use]
    pub fn metadata(&self) -> &HashMap<String, String> {
        &self.metadata
    }

    /// Set the visualization color for this region.
    #[must_use]
    pub fn with_color(mut self, r: u8, g: u8, b: u8) -> Self {
        self.color = Some((r, g, b));
        self
    }

    /// Get the visualization color.
    #[must_use]
    pub fn color(&self) -> Option<(u8, u8, u8)> {
        self.color
    }

    /// Compute the union with another region.
    ///
    /// Returns a new region containing elements from both regions.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::MeshRegion;
    ///
    /// let r1 = MeshRegion::from_vertices("a", [0, 1, 2]);
    /// let r2 = MeshRegion::from_vertices("b", [2, 3, 4]);
    /// let union = r1.union(&r2);
    ///
    /// assert_eq!(union.vertex_count(), 5);
    /// ```
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        Self {
            name: format!("{}+{}", self.name, other.name),
            vertices: self.vertices.union(&other.vertices).copied().collect(),
            faces: self.faces.union(&other.faces).copied().collect(),
            metadata: HashMap::new(),
            color: self.color.or(other.color),
        }
    }

    /// Compute the intersection with another region.
    ///
    /// Returns a new region containing only elements in both regions.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::MeshRegion;
    ///
    /// let r1 = MeshRegion::from_vertices("a", [0, 1, 2]);
    /// let r2 = MeshRegion::from_vertices("b", [2, 3, 4]);
    /// let intersection = r1.intersection(&r2);
    ///
    /// assert_eq!(intersection.vertex_count(), 1);
    /// assert!(intersection.contains_vertex(2));
    /// ```
    #[must_use]
    pub fn intersection(&self, other: &Self) -> Self {
        Self {
            name: format!("{}&{}", self.name, other.name),
            vertices: self
                .vertices
                .intersection(&other.vertices)
                .copied()
                .collect(),
            faces: self.faces.intersection(&other.faces).copied().collect(),
            metadata: HashMap::new(),
            color: self.color.or(other.color),
        }
    }

    /// Compute the difference (self - other).
    ///
    /// Returns a new region containing elements in self but not in other.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::MeshRegion;
    ///
    /// let r1 = MeshRegion::from_vertices("a", [0, 1, 2]);
    /// let r2 = MeshRegion::from_vertices("b", [2, 3, 4]);
    /// let diff = r1.difference(&r2);
    ///
    /// assert_eq!(diff.vertex_count(), 2);
    /// assert!(diff.contains_vertex(0));
    /// assert!(diff.contains_vertex(1));
    /// assert!(!diff.contains_vertex(2));
    /// ```
    #[must_use]
    pub fn difference(&self, other: &Self) -> Self {
        Self {
            name: format!("{}-{}", self.name, other.name),
            vertices: self.vertices.difference(&other.vertices).copied().collect(),
            faces: self.faces.difference(&other.faces).copied().collect(),
            metadata: HashMap::new(),
            color: self.color,
        }
    }

    /// Compute the symmetric difference with another region.
    ///
    /// Returns a new region containing elements in either region but not both.
    #[must_use]
    pub fn symmetric_difference(&self, other: &Self) -> Self {
        Self {
            name: format!("{}^{}", self.name, other.name),
            vertices: self
                .vertices
                .symmetric_difference(&other.vertices)
                .copied()
                .collect(),
            faces: self
                .faces
                .symmetric_difference(&other.faces)
                .copied()
                .collect(),
            metadata: HashMap::new(),
            color: self.color.or(other.color),
        }
    }

    /// Clear all vertices and faces from this region.
    pub fn clear(&mut self) {
        self.vertices.clear();
        self.faces.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_region() {
        let region = MeshRegion::new("test");
        assert_eq!(region.name(), "test");
        assert!(region.is_empty());
        assert_eq!(region.vertex_count(), 0);
        assert_eq!(region.face_count(), 0);
    }

    #[test]
    fn test_from_vertices() {
        let region = MeshRegion::from_vertices("vertices", [0, 1, 2, 3]);
        assert_eq!(region.vertex_count(), 4);
        assert!(region.contains_vertex(0));
        assert!(region.contains_vertex(3));
        assert!(!region.contains_vertex(4));
    }

    #[test]
    fn test_from_faces() {
        let region = MeshRegion::from_faces("faces", [0, 1]);
        assert_eq!(region.face_count(), 2);
        assert!(region.contains_face(0));
        assert!(region.contains_face(1));
        assert!(!region.contains_face(2));
    }

    #[test]
    fn test_add_remove() {
        let mut region = MeshRegion::new("test");

        region.add_vertex(5);
        assert!(region.contains_vertex(5));

        region.add_face(10);
        assert!(region.contains_face(10));

        region.remove_vertex(5);
        assert!(!region.contains_vertex(5));

        region.remove_face(10);
        assert!(!region.contains_face(10));
    }

    #[test]
    fn test_union() {
        let r1 = MeshRegion::from_vertices("a", [0, 1, 2]);
        let r2 = MeshRegion::from_vertices("b", [2, 3, 4]);
        let union = r1.union(&r2);

        assert_eq!(union.vertex_count(), 5);
        assert!(union.contains_vertex(0));
        assert!(union.contains_vertex(4));
    }

    #[test]
    fn test_intersection() {
        let r1 = MeshRegion::from_vertices("a", [0, 1, 2]);
        let r2 = MeshRegion::from_vertices("b", [2, 3, 4]);
        let intersection = r1.intersection(&r2);

        assert_eq!(intersection.vertex_count(), 1);
        assert!(intersection.contains_vertex(2));
    }

    #[test]
    fn test_difference() {
        let r1 = MeshRegion::from_vertices("a", [0, 1, 2]);
        let r2 = MeshRegion::from_vertices("b", [2, 3, 4]);
        let diff = r1.difference(&r2);

        assert_eq!(diff.vertex_count(), 2);
        assert!(diff.contains_vertex(0));
        assert!(diff.contains_vertex(1));
        assert!(!diff.contains_vertex(2));
    }

    #[test]
    fn test_metadata() {
        let mut region = MeshRegion::new("test");
        region.set_metadata("material", "TPU-95A");
        region.set_metadata("thickness", "2.5");

        assert_eq!(region.get_metadata("material"), Some("TPU-95A"));
        assert_eq!(region.get_metadata("thickness"), Some("2.5"));
        assert_eq!(region.get_metadata("unknown"), None);
    }

    #[test]
    fn test_color() {
        let region = MeshRegion::new("test").with_color(255, 128, 0);
        assert_eq!(region.color(), Some((255, 128, 0)));
    }

    #[test]
    fn test_clear() {
        let mut region = MeshRegion::from_vertices("test", [0, 1, 2]);
        region.add_faces([0, 1]);
        assert!(!region.is_empty());

        region.clear();
        assert!(region.is_empty());
    }

    #[test]
    fn test_symmetric_difference() {
        let r1 = MeshRegion::from_vertices("a", [0, 1, 2]);
        let r2 = MeshRegion::from_vertices("b", [2, 3, 4]);
        let sym_diff = r1.symmetric_difference(&r2);

        assert_eq!(sym_diff.vertex_count(), 4);
        assert!(sym_diff.contains_vertex(0));
        assert!(sym_diff.contains_vertex(1));
        assert!(!sym_diff.contains_vertex(2)); // In both
        assert!(sym_diff.contains_vertex(3));
        assert!(sym_diff.contains_vertex(4));
    }
}
