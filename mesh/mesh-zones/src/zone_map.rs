//! Zone map for storing face-to-zone assignments.
//!
//! A zone map assigns each face in a mesh to a zone ID.

use hashbrown::{HashMap, HashSet};

use crate::error::{ZoneError, ZoneResult};

/// A map from face indices to zone IDs.
///
/// Zone ID 0 is reserved for "unassigned" faces.
#[derive(Debug, Clone)]
pub struct ZoneMap {
    /// Zone assignment for each face (0 = unassigned).
    zones: Vec<u32>,
}

impl ZoneMap {
    /// Create a new zone map with all faces unassigned.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_zones::ZoneMap;
    ///
    /// let map = ZoneMap::new(100);
    /// assert_eq!(map.face_count(), 100);
    /// assert!(map.all_unassigned());
    /// ```
    #[must_use]
    pub fn new(face_count: usize) -> Self {
        Self {
            zones: vec![0; face_count],
        }
    }

    /// Get the zone ID for a face.
    ///
    /// Returns `None` if the face index is out of bounds.
    #[must_use]
    pub fn get(&self, face_idx: usize) -> Option<u32> {
        self.zones.get(face_idx).copied()
    }

    /// Set the zone ID for a face.
    ///
    /// # Errors
    ///
    /// Returns an error if the face index is out of bounds.
    pub fn set(&mut self, face_idx: usize, zone_id: u32) -> ZoneResult<()> {
        if face_idx >= self.zones.len() {
            return Err(ZoneError::FaceOutOfBounds {
                face_idx,
                face_count: self.zones.len(),
            });
        }
        self.zones[face_idx] = zone_id;
        Ok(())
    }

    /// Check if a face is assigned to a zone.
    #[must_use]
    pub fn is_assigned(&self, face_idx: usize) -> bool {
        self.zones.get(face_idx).is_some_and(|&z| z != 0)
    }

    /// Get the number of faces.
    #[must_use]
    pub fn face_count(&self) -> usize {
        self.zones.len()
    }

    /// Check if all faces are unassigned.
    #[must_use]
    pub fn all_unassigned(&self) -> bool {
        self.zones.iter().all(|&z| z == 0)
    }

    /// Get the number of unique zones (excluding unassigned).
    #[must_use]
    pub fn zone_count(&self) -> usize {
        let unique: HashSet<u32> = self.zones.iter().filter(|&&z| z != 0).copied().collect();
        unique.len()
    }

    /// Get all unique zone IDs (excluding unassigned).
    #[must_use]
    pub fn zone_ids(&self) -> Vec<u32> {
        let unique: HashSet<u32> = self.zones.iter().filter(|&&z| z != 0).copied().collect();
        let mut ids: Vec<u32> = unique.into_iter().collect();
        ids.sort_unstable();
        ids
    }

    /// Get all face indices assigned to a specific zone.
    #[must_use]
    pub fn faces_in_zone(&self, zone_id: u32) -> Vec<usize> {
        self.zones
            .iter()
            .enumerate()
            .filter(|&(_, z)| *z == zone_id)
            .map(|(idx, _)| idx)
            .collect()
    }

    /// Get the number of faces in each zone.
    #[must_use]
    pub fn zone_sizes(&self) -> HashMap<u32, usize> {
        let mut sizes = HashMap::new();
        for &zone in &self.zones {
            if zone != 0 {
                *sizes.entry(zone).or_insert(0) += 1;
            }
        }
        sizes
    }

    /// Get all unassigned face indices.
    #[must_use]
    pub fn unassigned_faces(&self) -> Vec<usize> {
        self.zones
            .iter()
            .enumerate()
            .filter(|&(_, z)| *z == 0)
            .map(|(idx, _)| idx)
            .collect()
    }

    /// Clear all zone assignments.
    pub fn clear(&mut self) {
        self.zones.fill(0);
    }

    /// Assign multiple faces to a zone.
    ///
    /// # Errors
    ///
    /// Returns an error if any face index is out of bounds.
    pub fn assign_many(&mut self, face_indices: &[usize], zone_id: u32) -> ZoneResult<()> {
        for &face_idx in face_indices {
            self.set(face_idx, zone_id)?;
        }
        Ok(())
    }

    /// Get zone assignments as a slice.
    #[must_use]
    pub fn as_slice(&self) -> &[u32] {
        &self.zones
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_map() {
        let map = ZoneMap::new(10);
        assert_eq!(map.face_count(), 10);
        assert!(map.all_unassigned());
    }

    #[test]
    fn get_set() {
        let mut map = ZoneMap::new(10);
        assert!(map.set(5, 1).is_ok());
        assert_eq!(map.get(5), Some(1));
    }

    #[test]
    fn set_out_of_bounds() {
        let mut map = ZoneMap::new(10);
        assert!(map.set(100, 1).is_err());
    }

    #[test]
    fn is_assigned() {
        let mut map = ZoneMap::new(10);
        assert!(!map.is_assigned(5));
        map.set(5, 1).expect("should set");
        assert!(map.is_assigned(5));
    }

    #[test]
    fn zone_count() {
        let mut map = ZoneMap::new(10);
        map.set(0, 1).expect("should set");
        map.set(1, 1).expect("should set");
        map.set(2, 2).expect("should set");
        assert_eq!(map.zone_count(), 2);
    }

    #[test]
    fn zone_ids() {
        let mut map = ZoneMap::new(10);
        map.set(0, 3).expect("should set");
        map.set(1, 1).expect("should set");
        let ids = map.zone_ids();
        assert_eq!(ids, vec![1, 3]);
    }

    #[test]
    fn faces_in_zone() {
        let mut map = ZoneMap::new(10);
        map.set(0, 1).expect("should set");
        map.set(5, 1).expect("should set");
        map.set(9, 2).expect("should set");

        let faces = map.faces_in_zone(1);
        assert_eq!(faces, vec![0, 5]);
    }

    #[test]
    fn zone_sizes() {
        let mut map = ZoneMap::new(10);
        map.set(0, 1).expect("should set");
        map.set(1, 1).expect("should set");
        map.set(2, 2).expect("should set");

        let sizes = map.zone_sizes();
        assert_eq!(sizes.get(&1), Some(&2));
        assert_eq!(sizes.get(&2), Some(&1));
    }

    #[test]
    fn unassigned_faces() {
        let mut map = ZoneMap::new(5);
        map.set(1, 1).expect("should set");
        map.set(3, 1).expect("should set");

        let unassigned = map.unassigned_faces();
        assert_eq!(unassigned, vec![0, 2, 4]);
    }

    #[test]
    fn clear() {
        let mut map = ZoneMap::new(10);
        map.set(0, 1).expect("should set");
        map.set(5, 2).expect("should set");
        map.clear();
        assert!(map.all_unassigned());
    }

    #[test]
    fn assign_many() {
        let mut map = ZoneMap::new(10);
        assert!(map.assign_many(&[0, 1, 2], 5).is_ok());
        assert_eq!(map.get(0), Some(5));
        assert_eq!(map.get(1), Some(5));
        assert_eq!(map.get(2), Some(5));
    }

    #[test]
    fn assign_many_out_of_bounds() {
        let mut map = ZoneMap::new(10);
        assert!(map.assign_many(&[0, 100, 2], 5).is_err());
    }
}
