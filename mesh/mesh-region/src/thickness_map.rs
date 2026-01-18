//! Thickness mapping for mesh regions.
//!
//! A [`ThicknessMap`] assigns thickness values to vertices or faces,
//! enabling variable-thickness shell generation or other operations.

use hashbrown::HashSet;
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

use crate::region::MeshRegion;

/// A thickness map that assigns thickness values to vertices or faces.
///
/// Thickness values can be set per-vertex, per-face, or by region.
/// When querying thickness, face values take precedence over vertex values.
///
/// # Example
///
/// ```
/// use mesh_region::{ThicknessMap, MeshRegion};
///
/// let mut map = ThicknessMap::new(2.0);
///
/// // Set thickness for specific vertices
/// map.set_vertex_thickness(0, 3.0);
/// map.set_vertex_thickness(1, 4.0);
///
/// assert!((map.get_vertex_thickness(0) - 3.0).abs() < 1e-10);
/// assert!((map.get_vertex_thickness(2) - 2.0).abs() < 1e-10); // default
/// ```
#[derive(Debug, Clone)]
#[allow(clippy::struct_field_names)]
pub struct ThicknessMap {
    /// Per-vertex thickness values.
    vertex_thickness: hashbrown::HashMap<u32, f64>,

    /// Per-face thickness values (takes precedence over vertex if both set).
    face_thickness: hashbrown::HashMap<u32, f64>,

    /// Default thickness for vertices/faces not in the map.
    default_thickness: f64,
}

impl ThicknessMap {
    /// Create a new thickness map with a default thickness.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::ThicknessMap;
    ///
    /// let map = ThicknessMap::new(2.5);
    /// assert!((map.default_thickness() - 2.5).abs() < 1e-10);
    /// ```
    #[must_use]
    pub fn new(default_thickness: f64) -> Self {
        Self {
            vertex_thickness: hashbrown::HashMap::new(),
            face_thickness: hashbrown::HashMap::new(),
            default_thickness,
        }
    }

    /// Create a uniform thickness map.
    ///
    /// All vertices and faces will have the same thickness.
    #[must_use]
    pub fn uniform(thickness: f64) -> Self {
        Self::new(thickness)
    }

    /// Get the default thickness.
    #[must_use]
    pub fn default_thickness(&self) -> f64 {
        self.default_thickness
    }

    /// Set the default thickness.
    pub fn set_default_thickness(&mut self, thickness: f64) {
        self.default_thickness = thickness;
    }

    /// Set thickness for a vertex.
    pub fn set_vertex_thickness(&mut self, vertex_index: u32, thickness: f64) {
        self.vertex_thickness.insert(vertex_index, thickness);
    }

    /// Set thickness for a face.
    pub fn set_face_thickness(&mut self, face_index: u32, thickness: f64) {
        self.face_thickness.insert(face_index, thickness);
    }

    /// Set thickness for all vertices in a region.
    pub fn set_region_thickness(&mut self, region: &MeshRegion, thickness: f64) {
        for vi in region.vertices() {
            self.vertex_thickness.insert(vi, thickness);
        }
        for fi in region.faces() {
            self.face_thickness.insert(fi, thickness);
        }
    }

    /// Get the thickness at a vertex.
    ///
    /// Returns the explicitly set value, or the default if not set.
    #[must_use]
    pub fn get_vertex_thickness(&self, vertex_index: u32) -> f64 {
        self.vertex_thickness
            .get(&vertex_index)
            .copied()
            .unwrap_or(self.default_thickness)
    }

    /// Get the thickness at a face.
    ///
    /// Returns the explicitly set value, or the default if not set.
    #[must_use]
    pub fn get_face_thickness(&self, face_index: u32) -> f64 {
        self.face_thickness
            .get(&face_index)
            .copied()
            .unwrap_or(self.default_thickness)
    }

    /// Get the thickness at a face, averaging vertex values if no face value is set.
    ///
    /// This is useful when you have per-vertex thickness but need a single
    /// value per face.
    #[must_use]
    pub fn get_face_thickness_averaged(&self, mesh: &IndexedMesh, face_index: u32) -> f64 {
        if let Some(&t) = self.face_thickness.get(&face_index) {
            return t;
        }

        if let Some(face) = mesh.faces.get(face_index as usize) {
            let t0 = self.get_vertex_thickness(face[0]);
            let t1 = self.get_vertex_thickness(face[1]);
            let t2 = self.get_vertex_thickness(face[2]);
            return (t0 + t1 + t2) / 3.0;
        }

        self.default_thickness
    }

    /// Check if a vertex has an explicit thickness set.
    #[must_use]
    pub fn has_vertex_thickness(&self, vertex_index: u32) -> bool {
        self.vertex_thickness.contains_key(&vertex_index)
    }

    /// Check if a face has an explicit thickness set.
    #[must_use]
    pub fn has_face_thickness(&self, face_index: u32) -> bool {
        self.face_thickness.contains_key(&face_index)
    }

    /// Get the number of vertices with explicit thickness values.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.vertex_thickness.len()
    }

    /// Get the number of faces with explicit thickness values.
    #[must_use]
    pub fn face_count(&self) -> usize {
        self.face_thickness.len()
    }

    /// Remove the explicit thickness value for a vertex.
    pub fn remove_vertex_thickness(&mut self, vertex_index: u32) -> Option<f64> {
        self.vertex_thickness.remove(&vertex_index)
    }

    /// Remove the explicit thickness value for a face.
    pub fn remove_face_thickness(&mut self, face_index: u32) -> Option<f64> {
        self.face_thickness.remove(&face_index)
    }

    /// Clear all explicit thickness values.
    pub fn clear(&mut self) {
        self.vertex_thickness.clear();
        self.face_thickness.clear();
    }

    /// Create a thickness map from regions with specified thicknesses.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_region::{ThicknessMap, MeshRegion};
    ///
    /// let thick = MeshRegion::from_vertices("thick", [0, 1]);
    /// let thin = MeshRegion::from_vertices("thin", [2, 3]);
    ///
    /// let map = ThicknessMap::from_regions(&[(thick, 5.0), (thin, 1.0)], 2.0);
    ///
    /// assert!((map.get_vertex_thickness(0) - 5.0).abs() < 1e-10);
    /// assert!((map.get_vertex_thickness(2) - 1.0).abs() < 1e-10);
    /// assert!((map.get_vertex_thickness(4) - 2.0).abs() < 1e-10); // default
    /// ```
    #[must_use]
    pub fn from_regions(regions: &[(MeshRegion, f64)], default_thickness: f64) -> Self {
        let mut map = Self::new(default_thickness);
        for (region, thickness) in regions {
            map.set_region_thickness(region, *thickness);
        }
        map
    }

    /// Create a gradient thickness map between two regions.
    ///
    /// Vertices between the regions will have interpolated thickness values.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, Vertex, Point3};
    /// use mesh_region::{ThicknessMap, MeshRegion};
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
    /// mesh.vertices.push(Vertex::new(Point3::new(5.0, 0.0, 0.0)));
    /// mesh.vertices.push(Vertex::new(Point3::new(10.0, 0.0, 0.0)));
    ///
    /// let from = MeshRegion::from_vertices("from", [0]);
    /// let to = MeshRegion::from_vertices("to", [2]);
    ///
    /// let map = ThicknessMap::gradient(&mesh, &from, 1.0, &to, 5.0, 2.0);
    ///
    /// // Middle vertex should have interpolated value
    /// let middle_thickness = map.get_vertex_thickness(1);
    /// assert!(middle_thickness > 1.0 && middle_thickness < 5.0);
    /// ```
    #[must_use]
    pub fn gradient(
        mesh: &IndexedMesh,
        from_region: &MeshRegion,
        from_thickness: f64,
        to_region: &MeshRegion,
        to_thickness: f64,
        default_thickness: f64,
    ) -> Self {
        let mut map = Self::new(default_thickness);

        // Set explicit thicknesses for the regions
        map.set_region_thickness(from_region, from_thickness);
        map.set_region_thickness(to_region, to_thickness);

        // For vertices not in either region, interpolate based on distance
        let from_vertices: HashSet<u32> = from_region.vertices().collect();
        let to_vertices: HashSet<u32> = to_region.vertices().collect();

        // Compute average positions of each region
        let from_centroid = compute_centroid(mesh, &from_vertices);
        let to_centroid = compute_centroid(mesh, &to_vertices);

        if let (Some(fc), Some(tc)) = (from_centroid, to_centroid) {
            let axis = tc - fc;
            let axis_len = axis.norm();

            if axis_len > 1e-10 {
                let axis_normalized = axis / axis_len;

                for (vi, vertex) in mesh.vertices.iter().enumerate() {
                    let vi = u32::try_from(vi).unwrap_or(u32::MAX);
                    if from_vertices.contains(&vi) || to_vertices.contains(&vi) {
                        continue;
                    }

                    // Project onto axis
                    let to_vertex = vertex.position - fc;
                    let t = to_vertex.dot(&axis_normalized) / axis_len;
                    let t_clamped = t.clamp(0.0, 1.0);

                    let thickness = from_thickness + t_clamped * (to_thickness - from_thickness);
                    map.set_vertex_thickness(vi, thickness);
                }
            }
        }

        map
    }

    /// Get the minimum thickness value in the map.
    #[must_use]
    pub fn min_thickness(&self) -> f64 {
        let vertex_min = self
            .vertex_thickness
            .values()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let face_min = self
            .face_thickness
            .values()
            .copied()
            .fold(f64::INFINITY, f64::min);

        vertex_min.min(face_min).min(self.default_thickness)
    }

    /// Get the maximum thickness value in the map.
    #[must_use]
    pub fn max_thickness(&self) -> f64 {
        let vertex_max = self
            .vertex_thickness
            .values()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        let face_max = self
            .face_thickness
            .values()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);

        vertex_max.max(face_max).max(self.default_thickness)
    }

    /// Scale all thickness values by a factor.
    pub fn scale(&mut self, factor: f64) {
        for t in self.vertex_thickness.values_mut() {
            *t *= factor;
        }
        for t in self.face_thickness.values_mut() {
            *t *= factor;
        }
        self.default_thickness *= factor;
    }

    /// Offset all thickness values by a constant.
    pub fn offset(&mut self, offset: f64) {
        for t in self.vertex_thickness.values_mut() {
            *t += offset;
        }
        for t in self.face_thickness.values_mut() {
            *t += offset;
        }
        self.default_thickness += offset;
    }

    /// Clamp all thickness values to a range.
    pub fn clamp(&mut self, min: f64, max: f64) {
        for t in self.vertex_thickness.values_mut() {
            *t = t.clamp(min, max);
        }
        for t in self.face_thickness.values_mut() {
            *t = t.clamp(min, max);
        }
        self.default_thickness = self.default_thickness.clamp(min, max);
    }
}

/// Compute the centroid of a set of vertices.
fn compute_centroid(mesh: &IndexedMesh, vertices: &HashSet<u32>) -> Option<Point3<f64>> {
    if vertices.is_empty() {
        return None;
    }

    let sum: Vector3<f64> = vertices
        .iter()
        .filter_map(|&vi| mesh.vertices.get(vi as usize))
        .map(|v| v.position.coords)
        .sum();

    #[allow(clippy::cast_precision_loss)]
    Some(Point3::from(sum / vertices.len() as f64))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    #[test]
    fn test_new_map() {
        let map = ThicknessMap::new(2.5);
        assert!((map.default_thickness() - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_vertex_thickness() {
        let mut map = ThicknessMap::new(2.0);

        map.set_vertex_thickness(0, 3.0);
        map.set_vertex_thickness(1, 4.0);

        assert!((map.get_vertex_thickness(0) - 3.0).abs() < 1e-10);
        assert!((map.get_vertex_thickness(1) - 4.0).abs() < 1e-10);
        assert!((map.get_vertex_thickness(2) - 2.0).abs() < 1e-10); // default
    }

    #[test]
    fn test_face_thickness() {
        let mut map = ThicknessMap::new(2.0);

        map.set_face_thickness(0, 5.0);

        assert!((map.get_face_thickness(0) - 5.0).abs() < 1e-10);
        assert!((map.get_face_thickness(1) - 2.0).abs() < 1e-10); // default
    }

    #[test]
    fn test_from_regions() {
        let r1 = MeshRegion::from_vertices("thick", [0, 1]);
        let r2 = MeshRegion::from_vertices("thin", [2, 3]);

        let map = ThicknessMap::from_regions(&[(r1, 5.0), (r2, 1.0)], 2.0);

        assert!((map.get_vertex_thickness(0) - 5.0).abs() < 1e-10);
        assert!((map.get_vertex_thickness(2) - 1.0).abs() < 1e-10);
        assert!((map.get_vertex_thickness(4) - 2.0).abs() < 1e-10); // default
    }

    #[test]
    fn test_face_thickness_averaged() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices
            .push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
        mesh.vertices
            .push(Vertex::new(Point3::new(1.0, 0.0, 0.0)));
        mesh.vertices
            .push(Vertex::new(Point3::new(0.5, 1.0, 0.0)));
        mesh.faces.push([0, 1, 2]);

        let mut map = ThicknessMap::new(2.0);
        map.set_vertex_thickness(0, 1.0);
        map.set_vertex_thickness(1, 2.0);
        map.set_vertex_thickness(2, 3.0);

        let avg = map.get_face_thickness_averaged(&mesh, 0);
        assert!((avg - 2.0).abs() < 1e-10); // (1 + 2 + 3) / 3 = 2
    }

    #[test]
    fn test_scale_and_offset() {
        let mut map = ThicknessMap::new(2.0);
        map.set_vertex_thickness(0, 4.0);

        map.scale(2.0);
        assert!((map.get_vertex_thickness(0) - 8.0).abs() < 1e-10);
        assert!((map.default_thickness() - 4.0).abs() < 1e-10);

        map.offset(-1.0);
        assert!((map.get_vertex_thickness(0) - 7.0).abs() < 1e-10);
        assert!((map.default_thickness() - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_clamp() {
        let mut map = ThicknessMap::new(2.0);
        map.set_vertex_thickness(0, 0.5);
        map.set_vertex_thickness(1, 10.0);

        map.clamp(1.0, 5.0);
        assert!((map.get_vertex_thickness(0) - 1.0).abs() < 1e-10);
        assert!((map.get_vertex_thickness(1) - 5.0).abs() < 1e-10);
        assert!((map.default_thickness() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_min_max_thickness() {
        let mut map = ThicknessMap::new(2.0);
        map.set_vertex_thickness(0, 1.0);
        map.set_vertex_thickness(1, 5.0);

        assert!((map.min_thickness() - 1.0).abs() < 1e-10);
        assert!((map.max_thickness() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_gradient() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices
            .push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
        mesh.vertices
            .push(Vertex::new(Point3::new(5.0, 0.0, 0.0)));
        mesh.vertices
            .push(Vertex::new(Point3::new(10.0, 0.0, 0.0)));

        let from = MeshRegion::from_vertices("from", [0]);
        let to = MeshRegion::from_vertices("to", [2]);

        let map = ThicknessMap::gradient(&mesh, &from, 1.0, &to, 5.0, 2.0);

        // Check endpoints
        assert!((map.get_vertex_thickness(0) - 1.0).abs() < 1e-10);
        assert!((map.get_vertex_thickness(2) - 5.0).abs() < 1e-10);

        // Middle vertex should have interpolated value (~3.0)
        let middle = map.get_vertex_thickness(1);
        assert!(middle > 2.0 && middle < 4.0);
    }
}
