//! Multi-part assembly management.
//!
//! The [`Assembly`] struct is the main container for managing multiple
//! mesh parts with hierarchical relationships, transforms, and connections.

use hashbrown::{HashMap, HashSet};
use mesh_types::{IndexedMesh, Point3};
use nalgebra::Isometry3;

use crate::bom::{BillOfMaterials, BomItem};
use crate::connection::Connection;
use crate::error::{AssemblyError, AssemblyResult};
use crate::part::{Part, compute_bbox};
use crate::validation::{AssemblyValidation, ClearanceResult, InterferenceResult};

/// A multi-part assembly.
///
/// An assembly manages a collection of parts with hierarchical parent-child
/// relationships, transforms, and connections between parts.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex, Point3};
/// use mesh_assembly::{Assembly, Part, Connection};
///
/// let mut assembly = Assembly::new("skate_boot");
///
/// // Create a boot mesh
/// let mut boot = IndexedMesh::new();
/// boot.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
/// boot.vertices.push(Vertex::new(Point3::new(10.0, 0.0, 0.0)));
/// boot.vertices.push(Vertex::new(Point3::new(5.0, 10.0, 0.0)));
/// boot.faces.push([0, 1, 2]);
///
/// // Add the boot as a part
/// assembly.add_part(Part::new("shell", boot.clone())).unwrap();
///
/// // Add liner as a child
/// assembly.add_part(
///     Part::new("liner", boot)
///         .with_parent("shell")
///         .with_translation(0.0, 0.0, 1.0)
/// ).unwrap();
///
/// assert_eq!(assembly.part_count(), 2);
/// ```
#[derive(Debug, Clone)]
pub struct Assembly {
    /// Assembly name/identifier.
    name: String,

    /// Parts in this assembly, keyed by ID.
    parts: HashMap<String, Part>,

    /// Connections between parts.
    connections: Vec<Connection>,

    /// Assembly-level metadata.
    metadata: HashMap<String, String>,

    /// Assembly version.
    version: Option<String>,
}

impl Assembly {
    /// Create a new empty assembly.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_assembly::Assembly;
    ///
    /// let assembly = Assembly::new("my_assembly");
    /// assert!(assembly.is_empty());
    /// ```
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            parts: HashMap::new(),
            connections: Vec::new(),
            metadata: HashMap::new(),
            version: None,
        }
    }

    /// Get the assembly name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Set the assembly name.
    pub fn set_name(&mut self, name: impl Into<String>) {
        self.name = name.into();
    }

    /// Get the assembly version.
    #[must_use]
    pub fn version(&self) -> Option<&str> {
        self.version.as_deref()
    }

    /// Set the assembly version.
    pub fn set_version(&mut self, version: impl Into<String>) {
        self.version = Some(version.into());
    }

    /// Get assembly metadata.
    #[must_use]
    pub fn metadata(&self) -> &HashMap<String, String> {
        &self.metadata
    }

    /// Get mutable assembly metadata.
    pub fn metadata_mut(&mut self) -> &mut HashMap<String, String> {
        &mut self.metadata
    }

    // =========================================================================
    // Part Management
    // =========================================================================

    /// Add a part to the assembly.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - A part with the same ID already exists
    /// - The specified parent part does not exist
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::IndexedMesh;
    /// use mesh_assembly::{Assembly, Part};
    ///
    /// let mut assembly = Assembly::new("test");
    /// assembly.add_part(Part::new("part1", IndexedMesh::new())).unwrap();
    /// assert_eq!(assembly.part_count(), 1);
    /// ```
    pub fn add_part(&mut self, part: Part) -> AssemblyResult<()> {
        if self.parts.contains_key(part.id()) {
            return Err(AssemblyError::DuplicatePart {
                id: part.id().to_string(),
            });
        }

        // Validate parent exists if specified
        if let Some(parent_id) = part.parent_id()
            && !self.parts.contains_key(parent_id)
        {
            return Err(AssemblyError::ParentNotFound {
                child_id: part.id().to_string(),
                parent_id: parent_id.to_string(),
            });
        }

        self.parts.insert(part.id().to_string(), part);
        Ok(())
    }

    /// Remove a part from the assembly.
    ///
    /// Returns the removed part, or `None` if not found.
    /// Also removes any connections involving this part and
    /// clears parent references from children.
    pub fn remove_part(&mut self, part_id: &str) -> Option<Part> {
        let part = self.parts.remove(part_id)?;

        // Remove connections involving this part
        self.connections.retain(|conn| !conn.involves_part(part_id));

        // Clear parent references from children
        for other_part in self.parts.values_mut() {
            if other_part.parent_id() == Some(part_id) {
                other_part.set_parent(None);
            }
        }

        Some(part)
    }

    /// Get a part by ID.
    #[must_use]
    pub fn get_part(&self, part_id: &str) -> Option<&Part> {
        self.parts.get(part_id)
    }

    /// Get a mutable reference to a part by ID.
    pub fn get_part_mut(&mut self, part_id: &str) -> Option<&mut Part> {
        self.parts.get_mut(part_id)
    }

    /// Check if a part exists.
    #[must_use]
    pub fn contains_part(&self, part_id: &str) -> bool {
        self.parts.contains_key(part_id)
    }

    /// Get an iterator over part IDs.
    pub fn part_ids(&self) -> impl Iterator<Item = &str> {
        self.parts.keys().map(String::as_str)
    }

    /// Get an iterator over all parts.
    pub fn parts(&self) -> impl Iterator<Item = &Part> {
        self.parts.values()
    }

    /// Get a mutable iterator over all parts.
    pub fn parts_mut(&mut self) -> impl Iterator<Item = &mut Part> {
        self.parts.values_mut()
    }

    /// Get the number of parts.
    #[must_use]
    pub fn part_count(&self) -> usize {
        self.parts.len()
    }

    /// Check if the assembly is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.parts.is_empty()
    }

    // =========================================================================
    // Hierarchy
    // =========================================================================

    /// Get child parts of a parent.
    #[must_use]
    pub fn get_children(&self, parent_id: &str) -> Vec<&Part> {
        self.parts
            .values()
            .filter(|p| p.parent_id() == Some(parent_id))
            .collect()
    }

    /// Get root parts (parts with no parent).
    #[must_use]
    pub fn get_root_parts(&self) -> Vec<&Part> {
        self.parts
            .values()
            .filter(|p| p.parent_id().is_none())
            .collect()
    }

    /// Compute the world transform for a part (including parent transforms).
    #[must_use]
    pub fn get_world_transform(&self, part_id: &str) -> Option<Isometry3<f64>> {
        let part = self.parts.get(part_id)?;
        let mut transform = *part.transform();

        // Walk up the parent chain
        let mut current_parent_id = part.parent_id();
        while let Some(parent_id) = current_parent_id {
            if let Some(parent) = self.parts.get(parent_id) {
                transform = parent.transform() * transform;
                current_parent_id = parent.parent_id();
            } else {
                break;
            }
        }

        Some(transform)
    }

    /// Get a transformed copy of a part's mesh (world coordinates).
    #[must_use]
    pub fn get_transformed_mesh(&self, part_id: &str) -> Option<IndexedMesh> {
        let part = self.parts.get(part_id)?;
        let world_transform = self.get_world_transform(part_id)?;

        let mut mesh = part.mesh().clone();
        for vertex in &mut mesh.vertices {
            vertex.position = world_transform * vertex.position;
        }

        Some(mesh)
    }

    /// Merge all parts into a single mesh (world coordinates).
    #[must_use]
    pub fn to_merged_mesh(&self) -> IndexedMesh {
        let mut result = IndexedMesh::new();

        for part_id in self.parts.keys() {
            if let Some(mesh) = self.get_transformed_mesh(part_id) {
                let vertex_offset = u32::try_from(result.vertices.len()).unwrap_or(u32::MAX);

                // Add vertices
                result.vertices.extend(mesh.vertices);

                // Add faces with offset
                for face in &mesh.faces {
                    result.faces.push([
                        face[0].saturating_add(vertex_offset),
                        face[1].saturating_add(vertex_offset),
                        face[2].saturating_add(vertex_offset),
                    ]);
                }
            }
        }

        result
    }

    // =========================================================================
    // Connections
    // =========================================================================

    /// Define a connection between two parts.
    ///
    /// # Errors
    ///
    /// Returns an error if either part does not exist.
    pub fn define_connection(&mut self, connection: Connection) -> AssemblyResult<()> {
        if !self.parts.contains_key(connection.from_part()) {
            return Err(AssemblyError::PartNotFound {
                id: connection.from_part().to_string(),
            });
        }
        if !self.parts.contains_key(connection.to_part()) {
            return Err(AssemblyError::PartNotFound {
                id: connection.to_part().to_string(),
            });
        }

        self.connections.push(connection);
        Ok(())
    }

    /// Get all connections.
    #[must_use]
    pub fn connections(&self) -> &[Connection] {
        &self.connections
    }

    /// Get connections for a specific part.
    #[must_use]
    pub fn connections_for_part(&self, part_id: &str) -> Vec<&Connection> {
        self.connections
            .iter()
            .filter(|c| c.involves_part(part_id))
            .collect()
    }

    /// Remove all connections involving a specific part.
    pub fn remove_connections_for_part(&mut self, part_id: &str) {
        self.connections.retain(|c| !c.involves_part(part_id));
    }

    // =========================================================================
    // Validation
    // =========================================================================

    /// Validate the assembly.
    #[must_use]
    pub fn validate(&self) -> AssemblyValidation {
        let mut result = AssemblyValidation::new();

        // Check for orphan parent references
        for part in self.parts.values() {
            if let Some(parent_id) = part.parent_id()
                && !self.parts.contains_key(parent_id)
            {
                result
                    .orphan_references
                    .push((part.id().to_string(), parent_id.to_string()));
            }
        }

        // Check for circular parent references
        for part in self.parts.values() {
            if self.has_circular_reference(part.id()) {
                result.circular_references.push(part.id().to_string());
            }
        }

        // Check connections
        for conn in &self.connections {
            if !self.parts.contains_key(conn.from_part()) {
                result
                    .invalid_connections
                    .push((conn.clone(), format!("Missing part: {}", conn.from_part())));
            }
            if !self.parts.contains_key(conn.to_part()) {
                result
                    .invalid_connections
                    .push((conn.clone(), format!("Missing part: {}", conn.to_part())));
            }
        }

        result
    }

    fn has_circular_reference(&self, part_id: &str) -> bool {
        let mut visited = HashSet::new();
        let mut current = Some(part_id);

        while let Some(id) = current {
            if visited.contains(id) {
                return true;
            }
            visited.insert(id);

            current = self.parts.get(id).and_then(|p| p.parent_id());
        }

        false
    }

    // =========================================================================
    // Interference & Clearance
    // =========================================================================

    /// Check interference between two parts.
    ///
    /// # Errors
    ///
    /// Returns an error if either part does not exist.
    pub fn check_interference(
        &self,
        part_a: &str,
        part_b: &str,
    ) -> AssemblyResult<InterferenceResult> {
        let mesh_a =
            self.get_transformed_mesh(part_a)
                .ok_or_else(|| AssemblyError::PartNotFound {
                    id: part_a.to_string(),
                })?;

        let mesh_b =
            self.get_transformed_mesh(part_b)
                .ok_or_else(|| AssemblyError::PartNotFound {
                    id: part_b.to_string(),
                })?;

        // Compute bounding boxes for quick rejection
        let bbox_a = compute_bbox(&mesh_a);
        let bbox_b = compute_bbox(&mesh_b);

        if !bboxes_overlap(&bbox_a, &bbox_b) {
            return Ok(InterferenceResult::no_interference(bbox_distance(
                &bbox_a, &bbox_b,
            )));
        }

        // For more detailed interference, we'd need proper mesh boolean ops
        // For now, report that bounding boxes overlap
        Ok(InterferenceResult::has_interference())
    }

    /// Check clearance between two parts.
    ///
    /// # Errors
    ///
    /// Returns an error if either part does not exist.
    pub fn check_clearance(
        &self,
        part_a: &str,
        part_b: &str,
        min_required: f64,
    ) -> AssemblyResult<ClearanceResult> {
        let mesh_a =
            self.get_transformed_mesh(part_a)
                .ok_or_else(|| AssemblyError::PartNotFound {
                    id: part_a.to_string(),
                })?;

        let mesh_b =
            self.get_transformed_mesh(part_b)
                .ok_or_else(|| AssemblyError::PartNotFound {
                    id: part_b.to_string(),
                })?;

        // Compute approximate clearance using bounding boxes
        let bbox_a = compute_bbox(&mesh_a);
        let bbox_b = compute_bbox(&mesh_b);

        let clearance = bbox_distance(&bbox_a, &bbox_b);

        Ok(ClearanceResult::new(clearance, min_required))
    }

    // =========================================================================
    // BOM
    // =========================================================================

    /// Generate a bill of materials (BOM) for the assembly.
    #[must_use]
    pub fn generate_bom(&self) -> BillOfMaterials {
        let mut bom = BillOfMaterials::new(&self.name);
        bom.version.clone_from(&self.version);
        bom.connections.clone_from(&self.connections);

        for (part_id, part) in &self.parts {
            let mesh = self
                .get_transformed_mesh(part_id)
                .unwrap_or_else(|| part.mesh().clone());
            let (min, max) = compute_bbox(&mesh);
            let dimensions = max - min;

            // Estimate volume (approximate using bounding box)
            let bbox_volume = dimensions.x * dimensions.y * dimensions.z;

            // Count triangles
            let triangle_count = mesh.faces.len();

            bom.items.push(BomItem {
                part_id: part_id.clone(),
                name: part_id.clone(),
                material: part.material().map(String::from),
                quantity: 1,
                dimensions: (dimensions.x, dimensions.y, dimensions.z),
                bounding_volume: bbox_volume,
                triangle_count,
                parent: part.parent_id().map(String::from),
                metadata: part.metadata().clone(),
            });
        }

        // Sort by part ID for consistent output
        bom.items.sort_by(|a, b| a.part_id.cmp(&b.part_id));

        bom
    }
}

impl Default for Assembly {
    fn default() -> Self {
        Self::new("Untitled Assembly")
    }
}

/// Check if two bounding boxes overlap.
fn bboxes_overlap(a: &(Point3<f64>, Point3<f64>), b: &(Point3<f64>, Point3<f64>)) -> bool {
    let (a_min, a_max) = a;
    let (b_min, b_max) = b;

    !(a_max.x < b_min.x
        || b_max.x < a_min.x
        || a_max.y < b_min.y
        || b_max.y < a_min.y
        || a_max.z < b_min.z
        || b_max.z < a_min.z)
}

/// Compute distance between two bounding boxes.
fn bbox_distance(a: &(Point3<f64>, Point3<f64>), b: &(Point3<f64>, Point3<f64>)) -> f64 {
    let (a_min, a_max) = a;
    let (b_min, b_max) = b;

    let dx = (b_min.x - a_max.x).max(a_min.x - b_max.x).max(0.0);
    let dy = (b_min.y - a_max.y).max(a_min.y - b_max.y).max(0.0);
    let dz = (b_min.z - a_max.z).max(a_min.z - b_max.z).max(0.0);

    (dx * dx + dy * dy + dz * dz).sqrt()
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
    fn test_assembly_new() {
        let assembly = Assembly::new("test_assembly");
        assert_eq!(assembly.name(), "test_assembly");
        assert!(assembly.is_empty());
        assert_eq!(assembly.part_count(), 0);
    }

    #[test]
    fn test_add_part() {
        let mut assembly = Assembly::new("test");
        let part = Part::new("part1", create_test_mesh());

        assembly.add_part(part).unwrap();
        assert_eq!(assembly.part_count(), 1);
        assert!(assembly.get_part("part1").is_some());
    }

    #[test]
    fn test_add_duplicate_part_fails() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("part1", create_test_mesh()))
            .unwrap();

        let result = assembly.add_part(Part::new("part1", create_test_mesh()));
        assert!(matches!(result, Err(AssemblyError::DuplicatePart { .. })));
    }

    #[test]
    fn test_add_part_missing_parent_fails() {
        let mut assembly = Assembly::new("test");
        let part = Part::new("child", create_test_mesh()).with_parent("nonexistent");

        let result = assembly.add_part(part);
        assert!(matches!(result, Err(AssemblyError::ParentNotFound { .. })));
    }

    #[test]
    fn test_remove_part() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("part1", create_test_mesh()))
            .unwrap();

        let removed = assembly.remove_part("part1");
        assert!(removed.is_some());
        assert!(assembly.is_empty());
    }

    #[test]
    fn test_remove_part_clears_connections() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("part1", create_test_mesh()))
            .unwrap();
        assembly
            .add_part(Part::new("part2", create_test_mesh()))
            .unwrap();
        assembly
            .define_connection(Connection::snap_fit("part1", "part2"))
            .unwrap();

        assembly.remove_part("part1");
        assert!(assembly.connections().is_empty());
    }

    #[test]
    fn test_parent_child() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("parent", create_test_mesh()))
            .unwrap();
        assembly
            .add_part(Part::new("child", create_test_mesh()).with_parent("parent"))
            .unwrap();

        let children = assembly.get_children("parent");
        assert_eq!(children.len(), 1);
        assert_eq!(children[0].id(), "child");
    }

    #[test]
    fn test_root_parts() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("root1", create_test_mesh()))
            .unwrap();
        assembly
            .add_part(Part::new("root2", create_test_mesh()))
            .unwrap();
        assembly
            .add_part(Part::new("child", create_test_mesh()).with_parent("root1"))
            .unwrap();

        let roots = assembly.get_root_parts();
        assert_eq!(roots.len(), 2);
    }

    #[test]
    fn test_world_transform() {
        let mut assembly = Assembly::new("test");

        let parent = Part::new("parent", create_test_mesh()).with_translation(10.0, 0.0, 0.0);
        assembly.add_part(parent).unwrap();

        let child = Part::new("child", create_test_mesh())
            .with_parent("parent")
            .with_translation(5.0, 0.0, 0.0);
        assembly.add_part(child).unwrap();

        let world_transform = assembly.get_world_transform("child").unwrap();
        assert!((world_transform.translation.vector.x - 15.0).abs() < 1e-10);
    }

    #[test]
    fn test_define_connection() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("part1", create_test_mesh()))
            .unwrap();
        assembly
            .add_part(Part::new("part2", create_test_mesh()))
            .unwrap();

        let conn = Connection::snap_fit("part1", "part2");
        assembly.define_connection(conn).unwrap();

        assert_eq!(assembly.connections().len(), 1);
    }

    #[test]
    fn test_connection_for_missing_part_fails() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("part1", create_test_mesh()))
            .unwrap();

        let conn = Connection::snap_fit("part1", "missing");
        let result = assembly.define_connection(conn);
        assert!(matches!(result, Err(AssemblyError::PartNotFound { .. })));
    }

    #[test]
    fn test_validate() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("part1", create_test_mesh()))
            .unwrap();

        let validation = assembly.validate();
        assert!(validation.is_valid());
    }

    #[test]
    fn test_to_merged_mesh() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("part1", create_test_mesh()))
            .unwrap();
        assembly
            .add_part(Part::new("part2", create_test_mesh()))
            .unwrap();

        let merged = assembly.to_merged_mesh();
        assert_eq!(merged.vertices.len(), 6); // 3 + 3
        assert_eq!(merged.faces.len(), 2); // 1 + 1
    }

    #[test]
    fn test_check_clearance() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("part1", create_test_mesh()).with_translation(0.0, 0.0, 0.0))
            .unwrap();
        assembly
            .add_part(Part::new("part2", create_test_mesh()).with_translation(10.0, 0.0, 0.0))
            .unwrap();

        let result = assembly.check_clearance("part1", "part2", 5.0).unwrap();
        assert!(result.meets_requirement);
        assert!(result.actual_clearance > 5.0);
    }

    #[test]
    fn test_generate_bom() {
        let mut assembly = Assembly::new("test_assembly");
        assembly.set_version("1.0");

        assembly
            .add_part(Part::new("part1", create_test_mesh()).with_material("PLA"))
            .unwrap();
        assembly
            .add_part(Part::new("part2", create_test_mesh()).with_material("TPU"))
            .unwrap();

        let bom = assembly.generate_bom();
        assert_eq!(bom.assembly_name, "test_assembly");
        assert_eq!(bom.version, Some("1.0".to_string()));
        assert_eq!(bom.items.len(), 2);
        assert_eq!(bom.total_parts(), 2);
    }

    #[test]
    fn test_bboxes_overlap() {
        let a = (Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let b = (Point3::new(0.5, 0.5, 0.5), Point3::new(1.5, 1.5, 1.5));
        assert!(bboxes_overlap(&a, &b));

        let c = (Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0));
        assert!(!bboxes_overlap(&a, &c));
    }

    #[test]
    fn test_bbox_distance() {
        let a = (Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        let b = (Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 1.0, 1.0));

        let dist = bbox_distance(&a, &b);
        assert!((dist - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_contains_part() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("part1", IndexedMesh::new()))
            .unwrap();

        assert!(assembly.contains_part("part1"));
        assert!(!assembly.contains_part("nonexistent"));
    }

    #[test]
    fn test_connections_for_part() {
        let mut assembly = Assembly::new("test");
        assembly
            .add_part(Part::new("a", IndexedMesh::new()))
            .unwrap();
        assembly
            .add_part(Part::new("b", IndexedMesh::new()))
            .unwrap();
        assembly
            .add_part(Part::new("c", IndexedMesh::new()))
            .unwrap();

        assembly
            .define_connection(Connection::snap_fit("a", "b"))
            .unwrap();
        assembly
            .define_connection(Connection::snap_fit("b", "c"))
            .unwrap();

        let conns = assembly.connections_for_part("b");
        assert_eq!(conns.len(), 2);

        let conns = assembly.connections_for_part("a");
        assert_eq!(conns.len(), 1);
    }
}
