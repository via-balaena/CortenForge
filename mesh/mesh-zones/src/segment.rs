//! Mesh segmentation algorithms.
//!
//! Automatically segments a mesh into regions based on geometry.

use mesh_types::IndexedMesh;
use nalgebra::Vector3;

use crate::adjacency::FaceAdjacency;
use crate::error::{ZoneError, ZoneResult};
use crate::grow::{grow_region, GrowConfig};
use crate::zone_map::ZoneMap;

/// Configuration for automatic segmentation.
#[derive(Debug, Clone)]
pub struct SegmentConfig {
    /// Maximum angle (in radians) between adjacent face normals.
    pub max_angle: f64,
    /// Minimum number of faces for a valid segment.
    pub min_faces: usize,
}

impl Default for SegmentConfig {
    fn default() -> Self {
        Self {
            max_angle: std::f64::consts::FRAC_PI_4, // 45 degrees
            min_faces: 1,
        }
    }
}

impl SegmentConfig {
    /// Create a config for coarse segmentation.
    #[must_use]
    pub fn coarse() -> Self {
        Self {
            max_angle: std::f64::consts::FRAC_PI_3, // 60 degrees
            min_faces: 10,
        }
    }

    /// Create a config for fine segmentation.
    #[must_use]
    pub fn fine() -> Self {
        Self {
            max_angle: 0.2, // ~11 degrees
            min_faces: 1,
        }
    }
}

/// Automatically segment a mesh into regions.
///
/// Uses region growing with normal-based criteria to find natural
/// surface regions.
///
/// # Arguments
///
/// * `mesh` - The mesh to segment
///
/// # Returns
///
/// A zone map with each face assigned to a segment.
///
/// # Errors
///
/// Returns an error if the mesh is empty.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_zones::{segment_mesh, SegmentConfig};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let zone_map = segment_mesh(&mesh, &SegmentConfig::default());
/// assert!(zone_map.is_ok());
/// ```
pub fn segment_mesh(mesh: &IndexedMesh, config: &SegmentConfig) -> ZoneResult<ZoneMap> {
    if mesh.faces.is_empty() {
        return Err(ZoneError::EmptyMesh);
    }

    let adjacency = FaceAdjacency::from_mesh(mesh);
    let mut zone_map = ZoneMap::new(mesh.faces.len());

    let grow_config = GrowConfig::default()
        .with_max_angle(config.max_angle)
        .ignore_boundaries();

    let mut next_zone_id: u32 = 1;

    // Process each unassigned face
    for face_idx in 0..mesh.faces.len() {
        if zone_map.is_assigned(face_idx) {
            continue;
        }

        // Grow a region from this face
        let count = grow_region(
            mesh,
            &adjacency,
            &mut zone_map,
            &[face_idx],
            next_zone_id,
            &grow_config,
        )?;

        // Only increment zone ID if segment is large enough
        if count >= config.min_faces {
            next_zone_id += 1;
        } else {
            // Unassign small segments (they'll be merged later or stay at this zone)
            // For now, we keep them assigned to allow iteration
            next_zone_id += 1;
        }
    }

    Ok(zone_map)
}

/// Segment mesh by connected components.
///
/// Each disconnected piece gets its own zone.
///
/// # Arguments
///
/// * `mesh` - The mesh to segment
///
/// # Returns
///
/// A zone map with each connected component having a unique zone ID.
///
/// # Errors
///
/// Returns an error if the mesh is empty.
pub fn segment_by_components(mesh: &IndexedMesh) -> ZoneResult<ZoneMap> {
    if mesh.faces.is_empty() {
        return Err(ZoneError::EmptyMesh);
    }

    let adjacency = FaceAdjacency::from_mesh(mesh);
    let components = adjacency.connected_components();

    let mut zone_map = ZoneMap::new(mesh.faces.len());

    for (i, component) in components.iter().enumerate() {
        let zone_id = (i + 1) as u32;
        for &face_idx in component {
            zone_map.set(face_idx, zone_id)?;
        }
    }

    Ok(zone_map)
}

/// Segment mesh by face normal direction.
///
/// Groups faces by their dominant normal direction (±X, ±Y, ±Z).
///
/// # Arguments
///
/// * `mesh` - The mesh to segment
///
/// # Returns
///
/// A zone map with faces grouped by normal direction.
/// Zone IDs: 1=+X, 2=-X, 3=+Y, 4=-Y, 5=+Z, 6=-Z
///
/// # Errors
///
/// Returns an error if the mesh is empty.
pub fn segment_by_normal_direction(mesh: &IndexedMesh) -> ZoneResult<ZoneMap> {
    if mesh.faces.is_empty() {
        return Err(ZoneError::EmptyMesh);
    }

    let mut zone_map = ZoneMap::new(mesh.faces.len());

    for (face_idx, face) in mesh.faces.iter().enumerate() {
        let v0 = &mesh.vertices[face[0] as usize].position;
        let v1 = &mesh.vertices[face[1] as usize].position;
        let v2 = &mesh.vertices[face[2] as usize].position;

        let e1 = *v1 - *v0;
        let e2 = *v2 - *v0;
        let normal = e1.cross(&e2);
        let normal = normal.try_normalize(f64::EPSILON).unwrap_or(Vector3::z());

        // Find dominant axis
        let abs_x = normal.x.abs();
        let abs_y = normal.y.abs();
        let abs_z = normal.z.abs();

        let zone_id = if abs_x >= abs_y && abs_x >= abs_z {
            if normal.x >= 0.0 { 1 } else { 2 }
        } else if abs_y >= abs_x && abs_y >= abs_z {
            if normal.y >= 0.0 { 3 } else { 4 }
        } else if normal.z >= 0.0 {
            5
        } else {
            6
        };

        zone_map.set(face_idx, zone_id)?;
    }

    Ok(zone_map)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn simple_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn cube_faces() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(-0.5, -0.5, -0.5));
        mesh.vertices.push(Vertex::from_coords(0.5, -0.5, -0.5));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, -0.5));
        mesh.vertices.push(Vertex::from_coords(-0.5, 0.5, -0.5));
        mesh.vertices.push(Vertex::from_coords(-0.5, -0.5, 0.5));
        mesh.vertices.push(Vertex::from_coords(0.5, -0.5, 0.5));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.5, 0.5));
        mesh.vertices.push(Vertex::from_coords(-0.5, 0.5, 0.5));

        // Bottom (z = -0.5, normal -Z)
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        // Top (z = 0.5, normal +Z)
        mesh.faces.push([4, 5, 6]);
        mesh.faces.push([4, 6, 7]);
        // Front (y = -0.5, normal -Y)
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        // Back (y = 0.5, normal +Y)
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        // Left (x = -0.5, normal -X)
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        // Right (x = 0.5, normal +X)
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    #[test]
    fn segment_config_presets() {
        let coarse = SegmentConfig::coarse();
        let fine = SegmentConfig::fine();

        assert!(coarse.max_angle > fine.max_angle);
        assert!(coarse.min_faces > fine.min_faces);
    }

    #[test]
    fn segment_empty_mesh() {
        let mesh = IndexedMesh::new();
        let result = segment_mesh(&mesh, &SegmentConfig::default());
        assert!(result.is_err());
    }

    #[test]
    fn segment_single_triangle() {
        let mesh = simple_triangle();
        let result = segment_mesh(&mesh, &SegmentConfig::default());

        assert!(result.is_ok());
        let zone_map = result.expect("zone map");
        assert_eq!(zone_map.zone_count(), 1);
    }

    #[test]
    fn segment_by_components_empty() {
        let mesh = IndexedMesh::new();
        let result = segment_by_components(&mesh);
        assert!(result.is_err());
    }

    #[test]
    fn segment_by_components_single() {
        let mesh = simple_triangle();
        let result = segment_by_components(&mesh);

        assert!(result.is_ok());
        let zone_map = result.expect("zone map");
        assert_eq!(zone_map.zone_count(), 1);
    }

    #[test]
    fn segment_by_normal_direction_empty() {
        let mesh = IndexedMesh::new();
        let result = segment_by_normal_direction(&mesh);
        assert!(result.is_err());
    }

    #[test]
    fn segment_by_normal_direction_cube() {
        let mesh = cube_faces();
        let result = segment_by_normal_direction(&mesh);

        assert!(result.is_ok());
        let zone_map = result.expect("zone map");
        // A cube should have 6 different normal directions
        assert_eq!(zone_map.zone_count(), 6);
    }

    #[test]
    fn segment_cube_by_angle() {
        let mesh = cube_faces();
        // With small angle tolerance, cube should segment into 6 regions
        let result = segment_mesh(&mesh, &SegmentConfig::fine());

        assert!(result.is_ok());
        let zone_map = result.expect("zone map");
        // Each face of the cube should be its own segment
        assert!(zone_map.zone_count() >= 6);
    }
}
