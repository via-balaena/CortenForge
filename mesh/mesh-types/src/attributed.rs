//! Attributed mesh — geometry with per-vertex domain attributes (`SoA` layout).

use std::collections::BTreeMap;

use cf_geometry::{Aabb, Bounded, IndexedMesh};
use nalgebra::Vector3;

use crate::VertexColor;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Returned from [`AttributedMesh::insert_extra`] when the supplied value
/// vector does not match the mesh's vertex count.
///
/// Marked `#[non_exhaustive]` so future variants of attribute-validation
/// failure (e.g., reserved names) can be added additively.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct AttributeMismatchError {
    /// Name of the attribute being inserted.
    pub name: String,
    /// Expected length (`vertex_count()`).
    pub expected: usize,
    /// Actual length supplied.
    pub actual: usize,
}

impl core::fmt::Display for AttributeMismatchError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "extra attribute `{}` has {} values but mesh has {} vertices",
            self.name, self.actual, self.expected
        )
    }
}

impl std::error::Error for AttributeMismatchError {}

/// A mesh with domain-specific per-vertex attributes.
///
/// Wraps [`cf_geometry::IndexedMesh`] (positions + faces) with optional
/// attribute arrays in struct-of-arrays (`SoA`) layout. Each attribute array,
/// when present, has length equal to `geometry.vertex_count()`.
///
/// # `SoA` Design
///
/// Attributes are stored as parallel arrays rather than per-vertex structs:
/// - Better cache performance for position-only iteration (most algorithms)
/// - Zero memory overhead when attributes are absent
/// - Each attribute can be independently present or absent
///
/// # Example
///
/// ```
/// use mesh_types::{AttributedMesh, unit_cube};
///
/// let geometry = unit_cube();
/// let mut mesh = AttributedMesh::new(geometry);
///
/// // Compute normals from geometry
/// mesh.compute_normals();
/// assert!(mesh.normals.is_some());
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AttributedMesh {
    /// The underlying geometry (positions + faces).
    pub geometry: IndexedMesh,

    /// Per-vertex unit normals. Length must equal `geometry.vertex_count()`.
    pub normals: Option<Vec<Vector3<f64>>>,

    /// Per-vertex colors. Length must equal `geometry.vertex_count()`.
    pub colors: Option<Vec<VertexColor>>,

    /// Per-vertex zone identifiers (e.g., anatomical region, material zone).
    /// Length must equal `geometry.vertex_count()`.
    pub zone_ids: Option<Vec<u32>>,

    /// Per-vertex clearance distance in millimeters.
    /// Length must equal `geometry.vertex_count()`.
    pub clearances: Option<Vec<f32>>,

    /// Per-vertex offset for variable shell thickness.
    /// Length must equal `geometry.vertex_count()`.
    pub offsets: Option<Vec<f32>>,

    /// Per-vertex texture coordinates (U, V).
    /// Length must equal `geometry.vertex_count()`.
    pub uvs: Option<Vec<(f32, f32)>>,

    /// Extensible per-vertex scalar attributes, keyed by name.
    ///
    /// Each value vector has length equal to `geometry.vertex_count()`.
    /// Use [`Self::insert_extra`] for length-validated insertion; direct
    /// mutation requires the caller to maintain that invariant.
    ///
    /// `BTreeMap` is used (over `HashMap`) so iteration order is
    /// deterministic — important for serialization, golden-file tests,
    /// and PLY field-data export.
    pub extras: BTreeMap<String, Vec<f32>>,
}

impl AttributedMesh {
    /// Create an `AttributedMesh` from geometry with no attributes.
    #[inline]
    #[must_use]
    pub const fn new(geometry: IndexedMesh) -> Self {
        Self {
            geometry,
            normals: None,
            colors: None,
            zone_ids: None,
            clearances: None,
            offsets: None,
            uvs: None,
            extras: BTreeMap::new(),
        }
    }

    /// Insert a per-vertex extra attribute, validating its length.
    ///
    /// On success, the attribute is stored under `name` and any previous
    /// value at that key is replaced.
    ///
    /// # Errors
    ///
    /// Returns [`AttributeMismatchError`] if `values.len()` does not equal
    /// `self.vertex_count()`. The mesh is left unchanged on error.
    pub fn insert_extra(
        &mut self,
        name: impl Into<String>,
        values: Vec<f32>,
    ) -> Result<(), AttributeMismatchError> {
        let name = name.into();
        let expected = self.vertex_count();
        let actual = values.len();
        if actual != expected {
            return Err(AttributeMismatchError {
                name,
                expected,
                actual,
            });
        }
        self.extras.insert(name, values);
        Ok(())
    }

    /// Returns the number of vertices.
    #[inline]
    #[must_use]
    pub const fn vertex_count(&self) -> usize {
        self.geometry.vertex_count()
    }

    /// Returns the number of faces.
    #[inline]
    #[must_use]
    pub const fn face_count(&self) -> usize {
        self.geometry.face_count()
    }

    /// Returns `true` if the geometry has no vertices or faces.
    #[inline]
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.geometry.is_empty()
    }

    /// Compute area-weighted vertex normals from geometry and store them.
    pub fn compute_normals(&mut self) {
        self.normals = Some(self.geometry.compute_vertex_normals());
    }

    /// Clear all stored normals.
    pub fn clear_normals(&mut self) {
        self.normals = None;
    }

    /// Flip all face normals by reversing winding order.
    ///
    /// Also negates stored vertex normals if present.
    pub fn flip_normals(&mut self) {
        for face in &mut self.geometry.faces {
            face.swap(1, 2);
        }
        if let Some(ref mut normals) = self.normals {
            for n in normals.iter_mut() {
                *n = -*n;
            }
        }
    }

    /// Rotate mesh 90 degrees around X axis (Y becomes Z, Z becomes -Y).
    ///
    /// Transforms both positions and stored normals.
    pub fn rotate_x_90(&mut self) {
        for v in &mut self.geometry.vertices {
            let old_y = v.y;
            let old_z = v.z;
            v.y = -old_z;
            v.z = old_y;
        }
        if let Some(ref mut normals) = self.normals {
            for n in normals.iter_mut() {
                let old_y = n.y;
                let old_z = n.z;
                n.y = -old_z;
                n.z = old_y;
            }
        }
    }

    /// Translate mesh so minimum Z is at zero.
    pub fn place_on_z_zero(&mut self) {
        let bounds = self.geometry.aabb();
        if !bounds.is_empty() {
            let offset = -bounds.min.z;
            for v in &mut self.geometry.vertices {
                v.z += offset;
            }
        }
    }
}

impl Default for AttributedMesh {
    fn default() -> Self {
        Self::new(IndexedMesh::default())
    }
}

impl Bounded for AttributedMesh {
    fn aabb(&self) -> Aabb {
        self.geometry.aabb()
    }
}

impl From<IndexedMesh> for AttributedMesh {
    fn from(geometry: IndexedMesh) -> Self {
        Self::new(geometry)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::unit_cube;

    #[test]
    fn attributed_mesh_from_geometry() {
        let mesh = AttributedMesh::new(unit_cube());
        assert_eq!(mesh.vertex_count(), 8);
        assert_eq!(mesh.face_count(), 12);
        assert!(mesh.normals.is_none());
    }

    #[test]
    fn compute_normals() {
        let mut mesh = AttributedMesh::new(unit_cube());
        mesh.compute_normals();
        let normals = mesh.normals.as_ref().unwrap();
        assert_eq!(normals.len(), 8);
        for n in normals {
            let len = n.norm();
            assert!(
                (len - 1.0).abs() < 1e-10,
                "Normal length should be 1.0, got {len}"
            );
        }
    }

    #[test]
    fn flip_normals_reverses_winding() {
        let mut mesh = AttributedMesh::new(unit_cube());
        let original_face = mesh.geometry.faces[0];
        mesh.flip_normals();
        assert_eq!(mesh.geometry.faces[0][1], original_face[2]);
        assert_eq!(mesh.geometry.faces[0][2], original_face[1]);
    }

    #[test]
    fn flip_normals_negates_stored_normals() {
        let mut mesh = AttributedMesh::new(unit_cube());
        mesh.compute_normals();
        let original = mesh.normals.as_ref().unwrap()[0];
        mesh.flip_normals();
        let flipped = mesh.normals.as_ref().unwrap()[0];
        assert!((flipped.x + original.x).abs() < 1e-10);
        assert!((flipped.y + original.y).abs() < 1e-10);
        assert!((flipped.z + original.z).abs() < 1e-10);
    }

    #[test]
    fn clear_normals() {
        let mut mesh = AttributedMesh::new(unit_cube());
        mesh.compute_normals();
        assert!(mesh.normals.is_some());
        mesh.clear_normals();
        assert!(mesh.normals.is_none());
    }

    #[test]
    fn bounded_impl() {
        let mesh = AttributedMesh::new(unit_cube());
        let aabb = mesh.aabb();
        assert!(!aabb.is_empty());
    }

    #[test]
    fn from_indexed_mesh() {
        let geom = unit_cube();
        let mesh: AttributedMesh = geom.into();
        assert_eq!(mesh.vertex_count(), 8);
    }

    #[test]
    fn default_is_empty() {
        let mesh = AttributedMesh::default();
        assert!(mesh.is_empty());
    }

    #[test]
    fn extras_default_empty() {
        let mesh = AttributedMesh::new(unit_cube());
        assert!(mesh.extras.is_empty());
    }

    #[test]
    fn insert_extra_rejects_length_mismatch() {
        let mut mesh = AttributedMesh::new(unit_cube());
        let too_short = vec![0.0_f32; 4];
        let err = mesh.insert_extra("stress", too_short).unwrap_err();
        assert_eq!(err.name, "stress");
        assert_eq!(err.expected, 8);
        assert_eq!(err.actual, 4);
        assert!(mesh.extras.is_empty(), "mesh must be unchanged on error");
    }

    #[test]
    fn insert_extra_stores_multiple_distinct_keys() {
        let mut mesh = AttributedMesh::new(unit_cube());
        let stress: Vec<f32> = (0_u8..8).map(f32::from).collect();
        let temperature: Vec<f32> = (0_u8..8).map(|i| 100.0 + f32::from(i)).collect();
        mesh.insert_extra("stress", stress.clone()).unwrap();
        mesh.insert_extra("temperature", temperature.clone())
            .unwrap();
        assert_eq!(mesh.extras.len(), 2);
        assert_eq!(mesh.extras.get("stress"), Some(&stress));
        assert_eq!(mesh.extras.get("temperature"), Some(&temperature));
    }

    #[test]
    fn insert_extra_overwrites_existing_key() {
        let mut mesh = AttributedMesh::new(unit_cube());
        mesh.insert_extra("stress", vec![0.0_f32; 8]).unwrap();
        mesh.insert_extra("stress", vec![1.0_f32; 8]).unwrap();
        assert_eq!(mesh.extras.len(), 1);
        let stored = mesh.extras.get("stress").unwrap();
        assert!(stored.iter().all(|&v| (v - 1.0).abs() < f32::EPSILON));
    }

    #[test]
    fn insert_extra_accepts_empty_vec_on_zero_vertex_mesh() {
        let mut mesh = AttributedMesh::default();
        assert_eq!(mesh.vertex_count(), 0);
        mesh.insert_extra("k", Vec::new()).unwrap();
        assert_eq!(mesh.extras.len(), 1);
        assert!(mesh.extras["k"].is_empty());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn extras_round_trip_through_serde() {
        let mut mesh = AttributedMesh::new(unit_cube());
        let stress: Vec<f32> = (0_u8..8).map(|i| f32::from(i) * 0.5).collect();
        mesh.insert_extra("stress", stress.clone()).unwrap();

        let json = serde_json::to_string(&mesh).unwrap();
        let restored: AttributedMesh = serde_json::from_str(&json).unwrap();

        assert_eq!(restored.extras.len(), 1);
        assert_eq!(restored.extras.get("stress"), Some(&stress));
    }
}
