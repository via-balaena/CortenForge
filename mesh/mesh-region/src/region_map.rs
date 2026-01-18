//! Collection of named regions.
//!
//! A [`RegionMap`] stores multiple named regions and provides methods
//! for querying which regions contain specific elements.

use hashbrown::{HashMap, HashSet};
use mesh_types::IndexedMesh;

use crate::error::{RegionError, RegionResult};
use crate::region::MeshRegion;

/// A collection of named regions for a mesh.
///
/// The `RegionMap` provides efficient lookup by region name and can
/// answer queries about which regions contain specific vertices or faces.
///
/// # Example
///
/// ```
/// use mesh_region::{RegionMap, MeshRegion};
///
/// let mut regions = RegionMap::new();
///
/// let top = MeshRegion::from_vertices("top", [4, 5, 6, 7]);
/// let bottom = MeshRegion::from_vertices("bottom", [0, 1, 2, 3]);
///
/// regions.add(top);
/// regions.add(bottom);
///
/// assert_eq!(regions.len(), 2);
/// assert!(regions.contains("top"));
/// ```
#[derive(Debug, Clone, Default)]
pub struct RegionMap {
    /// Regions indexed by name.
    regions: HashMap<String, MeshRegion>,
}

impl RegionMap {
    /// Create a new empty region map.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::RegionMap;
    ///
    /// let regions = RegionMap::new();
    /// assert!(regions.is_empty());
    /// ```
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a region to the map.
    ///
    /// If a region with the same name already exists, it will be replaced.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::{RegionMap, MeshRegion};
    ///
    /// let mut regions = RegionMap::new();
    /// regions.add(MeshRegion::from_vertices("heel", [0, 1, 2]));
    /// assert!(regions.contains("heel"));
    /// ```
    pub fn add(&mut self, region: MeshRegion) {
        self.regions.insert(region.name().to_string(), region);
    }

    /// Add a region, returning an error if it already exists.
    ///
    /// # Errors
    ///
    /// Returns [`RegionError::DuplicateRegion`] if a region with the same name exists.
    pub fn add_unique(&mut self, region: MeshRegion) -> RegionResult<()> {
        if self.regions.contains_key(region.name()) {
            return Err(RegionError::DuplicateRegion {
                name: region.name().to_string(),
            });
        }
        self.regions.insert(region.name().to_string(), region);
        Ok(())
    }

    /// Get a region by name.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&MeshRegion> {
        self.regions.get(name)
    }

    /// Get a mutable reference to a region by name.
    pub fn get_mut(&mut self, name: &str) -> Option<&mut MeshRegion> {
        self.regions.get_mut(name)
    }

    /// Remove a region by name.
    ///
    /// Returns the removed region, or `None` if not found.
    pub fn remove(&mut self, name: &str) -> Option<MeshRegion> {
        self.regions.remove(name)
    }

    /// Check if a region with the given name exists.
    #[must_use]
    pub fn contains(&self, name: &str) -> bool {
        self.regions.contains_key(name)
    }

    /// Get the number of regions.
    #[must_use]
    pub fn len(&self) -> usize {
        self.regions.len()
    }

    /// Check if the map is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.regions.is_empty()
    }

    /// Get an iterator over region names.
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.regions.keys().map(String::as_str)
    }

    /// Get an iterator over regions.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &MeshRegion)> {
        self.regions.iter().map(|(k, v)| (k.as_str(), v))
    }

    /// Get a mutable iterator over regions.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (&str, &mut MeshRegion)> {
        self.regions.iter_mut().map(|(k, v)| (k.as_str(), v))
    }

    /// Get an iterator over all regions.
    pub fn regions(&self) -> impl Iterator<Item = &MeshRegion> {
        self.regions.values()
    }

    /// Find which region(s) contain a vertex.
    ///
    /// Returns a vector of region names that contain the given vertex.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::{RegionMap, MeshRegion};
    ///
    /// let mut regions = RegionMap::new();
    /// regions.add(MeshRegion::from_vertices("a", [0, 1, 2]));
    /// regions.add(MeshRegion::from_vertices("b", [2, 3, 4]));
    ///
    /// let containing = regions.regions_containing_vertex(2);
    /// assert_eq!(containing.len(), 2);
    /// ```
    #[must_use]
    pub fn regions_containing_vertex(&self, vertex_index: u32) -> Vec<&str> {
        self.regions
            .iter()
            .filter(|(_, r)| r.contains_vertex(vertex_index))
            .map(|(name, _)| name.as_str())
            .collect()
    }

    /// Find which region(s) contain a face.
    ///
    /// Returns a vector of region names that contain the given face.
    #[must_use]
    pub fn regions_containing_face(&self, face_index: u32) -> Vec<&str> {
        self.regions
            .iter()
            .filter(|(_, r)| r.contains_face(face_index))
            .map(|(name, _)| name.as_str())
            .collect()
    }

    /// Get vertices that are not in any region.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, Vertex, Point3};
    /// use mesh_region::{RegionMap, MeshRegion};
    ///
    /// let mut mesh = IndexedMesh::new();
    /// for i in 0..5 {
    ///     mesh.vertices.push(Vertex::new(Point3::new(i as f64, 0.0, 0.0)));
    /// }
    ///
    /// let mut regions = RegionMap::new();
    /// regions.add(MeshRegion::from_vertices("partial", [0, 1, 2]));
    ///
    /// let unassigned = regions.unassigned_vertices(&mesh);
    /// assert_eq!(unassigned.len(), 2);
    /// assert!(unassigned.contains(&3));
    /// assert!(unassigned.contains(&4));
    /// ```
    #[must_use]
    pub fn unassigned_vertices(&self, mesh: &IndexedMesh) -> HashSet<u32> {
        let assigned: HashSet<u32> = self
            .regions
            .values()
            .flat_map(MeshRegion::vertices)
            .collect();

        let vertex_count = u32::try_from(mesh.vertices.len()).unwrap_or(u32::MAX);
        (0..vertex_count)
            .filter(|i| !assigned.contains(i))
            .collect()
    }

    /// Get faces that are not in any region.
    #[must_use]
    pub fn unassigned_faces(&self, mesh: &IndexedMesh) -> HashSet<u32> {
        let assigned: HashSet<u32> = self.regions.values().flat_map(MeshRegion::faces).collect();

        let face_count = u32::try_from(mesh.faces.len()).unwrap_or(u32::MAX);
        (0..face_count).filter(|i| !assigned.contains(i)).collect()
    }

    /// Clear all regions.
    pub fn clear(&mut self) {
        self.regions.clear();
    }

    /// Rename a region.
    ///
    /// # Errors
    ///
    /// Returns [`RegionError::RegionNotFound`] if the old name doesn't exist,
    /// or [`RegionError::DuplicateRegion`] if the new name already exists.
    pub fn rename(&mut self, old_name: &str, new_name: impl Into<String>) -> RegionResult<()> {
        let new_name = new_name.into();

        if !self.regions.contains_key(old_name) {
            return Err(RegionError::RegionNotFound {
                name: old_name.to_string(),
            });
        }

        if old_name != new_name && self.regions.contains_key(&new_name) {
            return Err(RegionError::DuplicateRegion { name: new_name });
        }

        if let Some(region) = self.regions.remove(old_name) {
            // Create new region with the new name
            let mut new_region = MeshRegion::from_vertices(&new_name, region.vertices());

            // Copy faces
            for face in region.faces() {
                new_region.add_face(face);
            }

            // Copy metadata
            for (k, v) in region.metadata() {
                new_region.set_metadata(k.clone(), v.clone());
            }

            // Copy color if present
            if let Some((r, g, b)) = region.color() {
                new_region = new_region.with_color(r, g, b);
            }

            self.regions.insert(new_name, new_region);
        }

        Ok(())
    }
}

impl IntoIterator for RegionMap {
    type Item = (String, MeshRegion);
    type IntoIter = hashbrown::hash_map::IntoIter<String, MeshRegion>;

    fn into_iter(self) -> Self::IntoIter {
        self.regions.into_iter()
    }
}

impl<'a> IntoIterator for &'a RegionMap {
    type Item = (&'a String, &'a MeshRegion);
    type IntoIter = hashbrown::hash_map::Iter<'a, String, MeshRegion>;

    fn into_iter(self) -> Self::IntoIter {
        self.regions.iter()
    }
}

impl FromIterator<MeshRegion> for RegionMap {
    fn from_iter<I: IntoIterator<Item = MeshRegion>>(iter: I) -> Self {
        let mut map = Self::new();
        for region in iter {
            map.add(region);
        }
        map
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_map() {
        let map = RegionMap::new();
        assert!(map.is_empty());
        assert_eq!(map.len(), 0);
    }

    #[test]
    fn test_add_and_get() {
        let mut map = RegionMap::new();
        let region = MeshRegion::from_vertices("test", [0, 1, 2]);
        map.add(region);

        assert!(map.contains("test"));
        assert!(!map.contains("other"));

        let r = map.get("test");
        assert!(r.is_some());
        assert_eq!(r.map(MeshRegion::vertex_count), Some(3));
    }

    #[test]
    fn test_add_unique() {
        let mut map = RegionMap::new();
        map.add(MeshRegion::new("test"));

        let result = map.add_unique(MeshRegion::new("test"));
        assert!(matches!(result, Err(RegionError::DuplicateRegion { .. })));
    }

    #[test]
    fn test_remove() {
        let mut map = RegionMap::new();
        map.add(MeshRegion::from_vertices("test", [0, 1]));

        let removed = map.remove("test");
        assert!(removed.is_some());
        assert!(map.is_empty());
    }

    #[test]
    fn test_regions_containing_vertex() {
        let mut map = RegionMap::new();
        map.add(MeshRegion::from_vertices("a", [0, 1, 2]));
        map.add(MeshRegion::from_vertices("b", [2, 3, 4]));

        let containing = map.regions_containing_vertex(2);
        assert_eq!(containing.len(), 2);

        let containing = map.regions_containing_vertex(0);
        assert_eq!(containing.len(), 1);

        let containing = map.regions_containing_vertex(10);
        assert!(containing.is_empty());
    }

    #[test]
    fn test_from_iterator() {
        let regions = vec![
            MeshRegion::from_vertices("a", [0, 1]),
            MeshRegion::from_vertices("b", [2, 3]),
        ];

        let map: RegionMap = regions.into_iter().collect();
        assert_eq!(map.len(), 2);
        assert!(map.contains("a"));
        assert!(map.contains("b"));
    }

    #[test]
    fn test_rename() {
        let mut map = RegionMap::new();
        map.add(MeshRegion::from_vertices("old", [0, 1, 2]));

        map.rename("old", "new").ok();
        assert!(!map.contains("old"));
        assert!(map.contains("new"));
    }

    #[test]
    fn test_rename_not_found() {
        let mut map = RegionMap::new();
        let result = map.rename("nonexistent", "new");
        assert!(matches!(result, Err(RegionError::RegionNotFound { .. })));
    }
}
