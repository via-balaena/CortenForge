//! Material zones for multi-material meshes.
//!
//! Material zones associate regions with material names and properties,
//! enabling multi-material 3D printing and rendering.

use hashbrown::HashMap;

use crate::region::MeshRegion;

/// A material zone associates a mesh region with material properties.
///
/// Material zones are used for:
/// - Multi-material 3D printing (3MF export)
/// - Rendering with different materials
/// - Simulation with varying material properties
///
/// # Example
///
/// ```
/// use mesh_region::{MaterialZone, MeshRegion, MaterialProperties};
///
/// let region = MeshRegion::from_vertices("heel", [0, 1, 2, 3]);
/// let zone = MaterialZone::new(region, "TPU-95A")
///     .with_shore_hardness(95.0)
///     .with_density(1.2)
///     .with_color(255, 128, 0);
///
/// assert_eq!(zone.material_name(), "TPU-95A");
/// assert_eq!(zone.properties().shore_hardness, Some(95.0));
/// ```
#[derive(Debug, Clone)]
pub struct MaterialZone {
    /// The region this zone covers.
    region: MeshRegion,

    /// Material name or ID.
    material_name: String,

    /// Material properties.
    properties: MaterialProperties,
}

impl MaterialZone {
    /// Create a new material zone.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::{MaterialZone, MeshRegion};
    ///
    /// let region = MeshRegion::from_vertices("zone1", [0, 1, 2]);
    /// let zone = MaterialZone::new(region, "PLA");
    /// assert_eq!(zone.material_name(), "PLA");
    /// ```
    #[must_use]
    pub fn new(region: MeshRegion, material_name: impl Into<String>) -> Self {
        Self {
            region,
            material_name: material_name.into(),
            properties: MaterialProperties::default(),
        }
    }

    /// Get the region for this zone.
    #[must_use]
    pub fn region(&self) -> &MeshRegion {
        &self.region
    }

    /// Get a mutable reference to the region.
    pub fn region_mut(&mut self) -> &mut MeshRegion {
        &mut self.region
    }

    /// Get the material name.
    #[must_use]
    pub fn material_name(&self) -> &str {
        &self.material_name
    }

    /// Set the material name.
    pub fn set_material_name(&mut self, name: impl Into<String>) {
        self.material_name = name.into();
    }

    /// Get the material properties.
    #[must_use]
    pub fn properties(&self) -> &MaterialProperties {
        &self.properties
    }

    /// Get a mutable reference to the material properties.
    pub fn properties_mut(&mut self) -> &mut MaterialProperties {
        &mut self.properties
    }

    /// Set density property (builder pattern).
    #[must_use]
    pub fn with_density(mut self, density: f64) -> Self {
        self.properties.density = Some(density);
        self
    }

    /// Set elastic modulus property (builder pattern).
    #[must_use]
    pub fn with_elastic_modulus(mut self, modulus: f64) -> Self {
        self.properties.elastic_modulus = Some(modulus);
        self
    }

    /// Set shore hardness property (builder pattern).
    #[must_use]
    pub fn with_shore_hardness(mut self, hardness: f64) -> Self {
        self.properties.shore_hardness = Some(hardness);
        self
    }

    /// Set flexibility property (builder pattern).
    #[must_use]
    pub fn with_flexibility(mut self, flexibility: f64) -> Self {
        self.properties.flexibility = Some(flexibility.clamp(0.0, 1.0));
        self
    }

    /// Set color property (builder pattern).
    #[must_use]
    pub fn with_color(mut self, r: u8, g: u8, b: u8) -> Self {
        self.properties.color = Some((r, g, b));
        self
    }

    /// Set a custom property (builder pattern).
    #[must_use]
    pub fn with_custom(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.properties.custom.insert(key.into(), value.into());
        self
    }
}

/// Material properties for a zone.
///
/// All properties are optional. Use `None` for properties that don't apply.
#[derive(Debug, Clone, Default)]
pub struct MaterialProperties {
    /// Density in g/cm³.
    pub density: Option<f64>,

    /// Elastic modulus / stiffness in megapascals.
    pub elastic_modulus: Option<f64>,

    /// Shore hardness (A scale, 0-100).
    pub shore_hardness: Option<f64>,

    /// Flexibility factor (0-1, where 0=rigid, 1=very flexible).
    pub flexibility: Option<f64>,

    /// Color (RGB, 0-255).
    pub color: Option<(u8, u8, u8)>,

    /// Additional custom properties.
    pub custom: HashMap<String, String>,
}

impl MaterialProperties {
    /// Create empty material properties.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create properties for a rigid material.
    #[must_use]
    pub fn rigid() -> Self {
        Self {
            flexibility: Some(0.0),
            ..Default::default()
        }
    }

    /// Create properties for a flexible material.
    #[must_use]
    pub fn flexible(flexibility: f64) -> Self {
        Self {
            flexibility: Some(flexibility.clamp(0.0, 1.0)),
            ..Default::default()
        }
    }

    /// Check if a custom property exists.
    #[must_use]
    pub fn has_custom(&self, key: &str) -> bool {
        self.custom.contains_key(key)
    }

    /// Get a custom property.
    #[must_use]
    pub fn get_custom(&self, key: &str) -> Option<&str> {
        self.custom.get(key).map(String::as_str)
    }

    /// Set a custom property.
    pub fn set_custom(&mut self, key: impl Into<String>, value: impl Into<String>) {
        self.custom.insert(key.into(), value.into());
    }
}

/// A collection of material zones.
///
/// Manages multiple material zones and can resolve which material applies
/// to a given vertex or face.
#[derive(Debug, Clone, Default)]
pub struct MaterialMap {
    /// Zones ordered by priority (later zones override earlier ones).
    zones: Vec<MaterialZone>,

    /// Default material for unassigned regions.
    default_material: Option<String>,
}

impl MaterialMap {
    /// Create an empty material map.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a material map with a default material.
    #[must_use]
    pub fn with_default(material_name: impl Into<String>) -> Self {
        Self {
            zones: Vec::new(),
            default_material: Some(material_name.into()),
        }
    }

    /// Add a material zone.
    ///
    /// Later-added zones have higher priority and will override earlier ones
    /// for overlapping vertices/faces.
    pub fn add(&mut self, zone: MaterialZone) {
        self.zones.push(zone);
    }

    /// Get the number of zones.
    #[must_use]
    pub fn len(&self) -> usize {
        self.zones.len()
    }

    /// Check if the map is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.zones.is_empty()
    }

    /// Get an iterator over zones.
    pub fn iter(&self) -> impl Iterator<Item = &MaterialZone> {
        self.zones.iter()
    }

    /// Get a mutable iterator over zones.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut MaterialZone> {
        self.zones.iter_mut()
    }

    /// Get the material for a vertex.
    ///
    /// Returns the material from the highest-priority zone containing
    /// the vertex, or the default material if none.
    #[must_use]
    pub fn material_for_vertex(&self, vertex_index: u32) -> Option<&str> {
        // Search in reverse order (later zones have higher priority)
        for zone in self.zones.iter().rev() {
            if zone.region.contains_vertex(vertex_index) {
                return Some(&zone.material_name);
            }
        }
        self.default_material.as_deref()
    }

    /// Get the material for a face.
    ///
    /// Returns the material from the highest-priority zone containing
    /// the face, or the default material if none.
    #[must_use]
    pub fn material_for_face(&self, face_index: u32) -> Option<&str> {
        // Search in reverse order (later zones have higher priority)
        for zone in self.zones.iter().rev() {
            if zone.region.contains_face(face_index) {
                return Some(&zone.material_name);
            }
        }
        self.default_material.as_deref()
    }

    /// Get all unique material names used.
    #[must_use]
    pub fn material_names(&self) -> Vec<&str> {
        let mut names: Vec<&str> = self
            .zones
            .iter()
            .map(|z| z.material_name.as_str())
            .collect();
        names.sort_unstable();
        names.dedup();
        names
    }

    /// Get zones by material name.
    #[must_use]
    pub fn zones_for_material(&self, material_name: &str) -> Vec<&MaterialZone> {
        self.zones
            .iter()
            .filter(|z| z.material_name == material_name)
            .collect()
    }

    /// Remove all zones for a material.
    pub fn remove_material(&mut self, material_name: &str) {
        self.zones.retain(|z| z.material_name != material_name);
    }

    /// Clear all zones.
    pub fn clear(&mut self) {
        self.zones.clear();
    }

    /// Set the default material.
    pub fn set_default_material(&mut self, material_name: impl Into<String>) {
        self.default_material = Some(material_name.into());
    }

    /// Get the default material.
    #[must_use]
    pub fn default_material(&self) -> Option<&str> {
        self.default_material.as_deref()
    }
}

impl IntoIterator for MaterialMap {
    type Item = MaterialZone;
    type IntoIter = std::vec::IntoIter<MaterialZone>;

    fn into_iter(self) -> Self::IntoIter {
        self.zones.into_iter()
    }
}

impl<'a> IntoIterator for &'a MaterialMap {
    type Item = &'a MaterialZone;
    type IntoIter = std::slice::Iter<'a, MaterialZone>;

    fn into_iter(self) -> Self::IntoIter {
        self.zones.iter()
    }
}

impl FromIterator<MaterialZone> for MaterialMap {
    fn from_iter<I: IntoIterator<Item = MaterialZone>>(iter: I) -> Self {
        let mut map = Self::new();
        for zone in iter {
            map.add(zone);
        }
        map
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_material_zone_creation() {
        let region = MeshRegion::from_vertices("test", [0, 1, 2]);
        let zone = MaterialZone::new(region, "PLA");

        assert_eq!(zone.material_name(), "PLA");
        assert!(zone.properties().density.is_none());
    }

    #[test]
    fn test_material_zone_builder() {
        let region = MeshRegion::from_vertices("heel", [0, 1, 2]);
        let zone = MaterialZone::new(region, "TPU-95A")
            .with_shore_hardness(95.0)
            .with_density(1.2)
            .with_flexibility(0.8)
            .with_color(255, 128, 0)
            .with_custom("manufacturer", "Polymaker");

        assert_eq!(zone.material_name(), "TPU-95A");
        assert_eq!(zone.properties().shore_hardness, Some(95.0));
        assert_eq!(zone.properties().density, Some(1.2));
        assert_eq!(zone.properties().flexibility, Some(0.8));
        assert_eq!(zone.properties().color, Some((255, 128, 0)));
        assert_eq!(zone.properties().get_custom("manufacturer"), Some("Polymaker"));
    }

    #[test]
    fn test_material_properties_presets() {
        let rigid = MaterialProperties::rigid();
        assert_eq!(rigid.flexibility, Some(0.0));

        let flex = MaterialProperties::flexible(0.7);
        assert_eq!(flex.flexibility, Some(0.7));
    }

    #[test]
    fn test_material_map() {
        let mut map = MaterialMap::new();

        let zone1 = MaterialZone::new(MeshRegion::from_vertices("a", [0, 1, 2]), "PLA");
        let zone2 = MaterialZone::new(MeshRegion::from_vertices("b", [2, 3, 4]), "TPU");

        map.add(zone1);
        map.add(zone2);

        assert_eq!(map.len(), 2);
        assert_eq!(map.material_for_vertex(0), Some("PLA"));
        assert_eq!(map.material_for_vertex(2), Some("TPU")); // Later zone wins
        assert_eq!(map.material_for_vertex(4), Some("TPU"));
        assert_eq!(map.material_for_vertex(10), None);
    }

    #[test]
    fn test_material_map_default() {
        let mut map = MaterialMap::with_default("ABS");

        let zone = MaterialZone::new(MeshRegion::from_vertices("a", [0, 1]), "PLA");
        map.add(zone);

        assert_eq!(map.material_for_vertex(0), Some("PLA"));
        assert_eq!(map.material_for_vertex(10), Some("ABS")); // Default
    }

    #[test]
    fn test_material_names() {
        let mut map = MaterialMap::new();

        map.add(MaterialZone::new(MeshRegion::from_vertices("a", [0]), "PLA"));
        map.add(MaterialZone::new(MeshRegion::from_vertices("b", [1]), "TPU"));
        map.add(MaterialZone::new(MeshRegion::from_vertices("c", [2]), "PLA"));

        let names = map.material_names();
        assert_eq!(names.len(), 2);
        assert!(names.contains(&"PLA"));
        assert!(names.contains(&"TPU"));
    }

    #[test]
    fn test_zones_for_material() {
        let mut map = MaterialMap::new();

        map.add(MaterialZone::new(MeshRegion::from_vertices("a", [0]), "PLA"));
        map.add(MaterialZone::new(MeshRegion::from_vertices("b", [1]), "TPU"));
        map.add(MaterialZone::new(MeshRegion::from_vertices("c", [2]), "PLA"));

        let pla_zones = map.zones_for_material("PLA");
        assert_eq!(pla_zones.len(), 2);
    }

    #[test]
    fn test_from_iterator() {
        let zones = vec![
            MaterialZone::new(MeshRegion::from_vertices("a", [0]), "PLA"),
            MaterialZone::new(MeshRegion::from_vertices("b", [1]), "TPU"),
        ];

        let map: MaterialMap = zones.into_iter().collect();
        assert_eq!(map.len(), 2);
    }
}
