//! Individual parts in an assembly.
//!
//! A [`Part`] represents a single component with its mesh geometry,
//! transform, and metadata.

use hashbrown::HashMap;
use mesh_types::{IndexedMesh, Point3};
use nalgebra::{Isometry3, UnitQuaternion, Vector3};

/// A single part in an assembly.
///
/// Each part has a unique identifier, mesh geometry, and a transform
/// that positions it relative to its parent (or world space if no parent).
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex, Point3};
/// use mesh_assembly::Part;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
/// mesh.vertices.push(Vertex::new(Point3::new(1.0, 0.0, 0.0)));
/// mesh.vertices.push(Vertex::new(Point3::new(0.5, 1.0, 0.0)));
/// mesh.faces.push([0, 1, 2]);
///
/// let part = Part::new("shell", mesh)
///     .with_translation(10.0, 0.0, 0.0)
///     .with_material("PA12");
///
/// assert_eq!(part.id(), "shell");
/// assert_eq!(part.material(), Some("PA12"));
/// ```
#[derive(Debug, Clone)]
pub struct Part {
    /// Unique identifier for this part.
    id: String,

    /// The mesh geometry.
    mesh: IndexedMesh,

    /// Transform relative to parent (or world if no parent).
    transform: Isometry3<f64>,

    /// Parent part ID (if any).
    parent_id: Option<String>,

    /// Part metadata.
    metadata: HashMap<String, String>,

    /// Material name.
    material: Option<String>,

    /// Is this part visible?
    visible: bool,
}

impl Part {
    /// Create a new part with identity transform.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::IndexedMesh;
    /// use mesh_assembly::Part;
    ///
    /// let mesh = IndexedMesh::new();
    /// let part = Part::new("my_part", mesh);
    /// assert_eq!(part.id(), "my_part");
    /// ```
    #[must_use]
    pub fn new(id: impl Into<String>, mesh: IndexedMesh) -> Self {
        Self {
            id: id.into(),
            mesh,
            transform: Isometry3::identity(),
            parent_id: None,
            metadata: HashMap::new(),
            material: None,
            visible: true,
        }
    }

    /// Get the part ID.
    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    /// Get the mesh geometry.
    #[must_use]
    pub fn mesh(&self) -> &IndexedMesh {
        &self.mesh
    }

    /// Get a mutable reference to the mesh.
    pub fn mesh_mut(&mut self) -> &mut IndexedMesh {
        &mut self.mesh
    }

    /// Get the local transform.
    #[must_use]
    pub fn transform(&self) -> &Isometry3<f64> {
        &self.transform
    }

    /// Get a mutable reference to the transform.
    pub fn transform_mut(&mut self) -> &mut Isometry3<f64> {
        &mut self.transform
    }

    /// Get the parent part ID.
    #[must_use]
    pub fn parent_id(&self) -> Option<&str> {
        self.parent_id.as_deref()
    }

    /// Get the material name.
    #[must_use]
    pub fn material(&self) -> Option<&str> {
        self.material.as_deref()
    }

    /// Check if the part is visible.
    #[must_use]
    pub fn is_visible(&self) -> bool {
        self.visible
    }

    /// Get the metadata map.
    #[must_use]
    pub fn metadata(&self) -> &HashMap<String, String> {
        &self.metadata
    }

    /// Get a mutable reference to the metadata map.
    pub fn metadata_mut(&mut self) -> &mut HashMap<String, String> {
        &mut self.metadata
    }

    /// Set the parent part ID (builder pattern).
    #[must_use]
    pub fn with_parent(mut self, parent_id: impl Into<String>) -> Self {
        self.parent_id = Some(parent_id.into());
        self
    }

    /// Set the transform (builder pattern).
    #[must_use]
    pub fn with_transform(mut self, transform: Isometry3<f64>) -> Self {
        self.transform = transform;
        self
    }

    /// Set translation (builder pattern).
    #[must_use]
    pub fn with_translation(mut self, x: f64, y: f64, z: f64) -> Self {
        self.transform.translation.vector = Vector3::new(x, y, z);
        self
    }

    /// Set rotation from axis-angle (builder pattern).
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::IndexedMesh;
    /// use mesh_assembly::Part;
    /// use nalgebra::Vector3;
    /// use std::f64::consts::FRAC_PI_2;
    ///
    /// let part = Part::new("rotated", IndexedMesh::new())
    ///     .with_rotation(Vector3::z_axis().into_inner(), FRAC_PI_2);
    /// ```
    #[must_use]
    pub fn with_rotation(mut self, axis: Vector3<f64>, angle: f64) -> Self {
        if let Some(axis_unit) = nalgebra::Unit::try_new(axis, 1e-10) {
            self.transform.rotation = UnitQuaternion::from_axis_angle(&axis_unit, angle);
        }
        self
    }

    /// Set material name (builder pattern).
    #[must_use]
    pub fn with_material(mut self, material: impl Into<String>) -> Self {
        self.material = Some(material.into());
        self
    }

    /// Set visibility (builder pattern).
    #[must_use]
    pub fn with_visible(mut self, visible: bool) -> Self {
        self.visible = visible;
        self
    }

    /// Add metadata (builder pattern).
    #[must_use]
    pub fn with_metadata(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.metadata.insert(key.into(), value.into());
        self
    }

    /// Set the parent part ID.
    pub fn set_parent(&mut self, parent_id: Option<String>) {
        self.parent_id = parent_id;
    }

    /// Set the material name.
    pub fn set_material(&mut self, material: Option<String>) {
        self.material = material;
    }

    /// Set visibility.
    pub fn set_visible(&mut self, visible: bool) {
        self.visible = visible;
    }

    /// Get the local bounding box of this part.
    ///
    /// Returns `(min, max)` corners of the axis-aligned bounding box.
    #[must_use]
    pub fn bounding_box(&self) -> (Point3<f64>, Point3<f64>) {
        compute_bbox(&self.mesh)
    }
}

/// Compute the axis-aligned bounding box of a mesh.
pub(crate) fn compute_bbox(mesh: &IndexedMesh) -> (Point3<f64>, Point3<f64>) {
    if mesh.vertices.is_empty() {
        return (Point3::origin(), Point3::origin());
    }

    let first = mesh.vertices[0].position;
    let mut min = first;
    let mut max = first;

    for v in &mesh.vertices {
        min.x = min.x.min(v.position.x);
        min.y = min.y.min(v.position.y);
        min.z = min.z.min(v.position.z);
        max.x = max.x.max(v.position.x);
        max.y = max.y.max(v.position.y);
        max.z = max.z.max(v.position.z);
    }

    (min, max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_test_mesh() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(1.0, 0.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(0.5, 1.0, 0.0)));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    #[test]
    fn test_part_new() {
        let mesh = create_test_mesh();
        let part = Part::new("test", mesh);

        assert_eq!(part.id(), "test");
        assert!(part.parent_id().is_none());
        assert!(part.material().is_none());
        assert!(part.is_visible());
    }

    #[test]
    fn test_part_builder() {
        let part = Part::new("test", create_test_mesh())
            .with_parent("parent")
            .with_translation(1.0, 2.0, 3.0)
            .with_material("TPU")
            .with_visible(false)
            .with_metadata("key", "value");

        assert_eq!(part.parent_id(), Some("parent"));
        assert!((part.transform().translation.vector.x - 1.0).abs() < 1e-10);
        assert!((part.transform().translation.vector.y - 2.0).abs() < 1e-10);
        assert!((part.transform().translation.vector.z - 3.0).abs() < 1e-10);
        assert_eq!(part.material(), Some("TPU"));
        assert!(!part.is_visible());
        assert_eq!(part.metadata().get("key"), Some(&"value".to_string()));
    }

    #[test]
    fn test_part_rotation() {
        use std::f64::consts::FRAC_PI_2;

        let part = Part::new("rotated", IndexedMesh::new())
            .with_rotation(Vector3::z_axis().into_inner(), FRAC_PI_2);

        // Check rotation angle is approximately PI/2
        assert!((part.transform().rotation.angle() - FRAC_PI_2).abs() < 1e-10);
    }

    #[test]
    fn test_bounding_box() {
        let part = Part::new("test", create_test_mesh());
        let (min, max) = part.bounding_box();

        assert!((min.x - 0.0).abs() < 1e-10);
        assert!((min.y - 0.0).abs() < 1e-10);
        assert!((min.z - 0.0).abs() < 1e-10);
        assert!((max.x - 1.0).abs() < 1e-10);
        assert!((max.y - 1.0).abs() < 1e-10);
        assert!((max.z - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_empty_mesh_bbox() {
        let part = Part::new("empty", IndexedMesh::new());
        let (min, max) = part.bounding_box();

        assert!((min.x - 0.0).abs() < 1e-10);
        assert!((max.x - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_part_setters() {
        let mut part = Part::new("test", create_test_mesh());

        part.set_parent(Some("new_parent".to_string()));
        assert_eq!(part.parent_id(), Some("new_parent"));

        part.set_material(Some("PLA".to_string()));
        assert_eq!(part.material(), Some("PLA"));

        part.set_visible(false);
        assert!(!part.is_visible());
    }
}
