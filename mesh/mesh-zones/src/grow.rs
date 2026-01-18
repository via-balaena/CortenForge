//! Region growing for zone assignment.
//!
//! Grows regions from seed faces based on adjacency and criteria.

use hashbrown::HashSet;
use mesh_types::IndexedMesh;
use nalgebra::Vector3;

use crate::adjacency::FaceAdjacency;
use crate::error::{ZoneError, ZoneResult};
use crate::zone_map::ZoneMap;

/// Configuration for region growing.
#[derive(Debug, Clone)]
pub struct GrowConfig {
    /// Maximum angle (in radians) between face normals to grow across.
    pub max_angle: f64,
    /// Whether to respect existing zone boundaries.
    pub respect_boundaries: bool,
}

impl Default for GrowConfig {
    fn default() -> Self {
        Self {
            max_angle: std::f64::consts::FRAC_PI_4, // 45 degrees
            respect_boundaries: true,
        }
    }
}

impl GrowConfig {
    /// Create a config with a specific maximum angle.
    #[must_use]
    pub fn with_max_angle(mut self, radians: f64) -> Self {
        self.max_angle = radians;
        self
    }

    /// Create a config that ignores existing zone boundaries.
    #[must_use]
    pub fn ignore_boundaries(mut self) -> Self {
        self.respect_boundaries = false;
        self
    }

    /// Create a config for flat regions (small angle tolerance).
    #[must_use]
    pub fn flat() -> Self {
        Self {
            max_angle: 0.1, // ~5.7 degrees
            respect_boundaries: true,
        }
    }

    /// Create a config for smooth regions (moderate angle tolerance).
    #[must_use]
    pub fn smooth() -> Self {
        Self {
            max_angle: std::f64::consts::FRAC_PI_6, // 30 degrees
            respect_boundaries: true,
        }
    }

    /// Create a config that grows everywhere (ignore normals).
    #[must_use]
    pub fn flood_fill() -> Self {
        Self {
            max_angle: std::f64::consts::PI,
            respect_boundaries: false,
        }
    }
}

/// Grow a region from seed faces.
///
/// Starting from the seed faces, grows outward to adjacent faces that
/// meet the angle criteria.
///
/// # Arguments
///
/// * `mesh` - The mesh to grow on
/// * `adjacency` - Precomputed face adjacency
/// * `zone_map` - Zone map to update
/// * `seeds` - Initial seed face indices
/// * `zone_id` - Zone ID to assign to grown region
/// * `config` - Growth configuration
///
/// # Returns
///
/// The number of faces assigned to the zone.
///
/// # Errors
///
/// Returns an error if:
/// - The mesh is empty
/// - No seeds are provided
/// - Any seed face is out of bounds
pub fn grow_region(
    mesh: &IndexedMesh,
    adjacency: &FaceAdjacency,
    zone_map: &mut ZoneMap,
    seeds: &[usize],
    zone_id: u32,
    config: &GrowConfig,
) -> ZoneResult<usize> {
    if mesh.faces.is_empty() {
        return Err(ZoneError::EmptyMesh);
    }

    if seeds.is_empty() {
        return Err(ZoneError::NoSeeds);
    }

    // Validate seeds
    for &seed in seeds {
        if seed >= mesh.faces.len() {
            return Err(ZoneError::FaceOutOfBounds {
                face_idx: seed,
                face_count: mesh.faces.len(),
            });
        }
    }

    // Compute face normals
    let normals = compute_face_normals(mesh);

    // Initialize with seeds
    let mut frontier: Vec<usize> = seeds.to_vec();
    let mut grown = HashSet::new();

    let cos_threshold = config.max_angle.cos();

    while let Some(face_idx) = frontier.pop() {
        if grown.contains(&face_idx) {
            continue;
        }

        // Check if already assigned to a different zone
        if config.respect_boundaries
            && let Some(existing) = zone_map.get(face_idx)
            && existing != 0
            && existing != zone_id
        {
            continue;
        }

        grown.insert(face_idx);

        // Grow to neighbors
        for &neighbor in adjacency.neighbors(face_idx) {
            if grown.contains(&neighbor) {
                continue;
            }

            // Check angle criterion
            let dot = normals[face_idx].dot(&normals[neighbor]);
            if dot >= cos_threshold {
                // Check zone boundary if respecting
                if config.respect_boundaries
                    && let Some(existing) = zone_map.get(neighbor)
                    && existing != 0
                    && existing != zone_id
                {
                    continue;
                }
                frontier.push(neighbor);
            }
        }
    }

    // Assign zone
    for &face_idx in &grown {
        zone_map.set(face_idx, zone_id)?;
    }

    Ok(grown.len())
}

/// Grow regions from multiple seeds, each getting a unique zone.
///
/// # Arguments
///
/// * `mesh` - The mesh to grow on
/// * `adjacency` - Precomputed face adjacency
/// * `zone_map` - Zone map to update
/// * `seed_groups` - Groups of seed faces, each group gets one zone
/// * `config` - Growth configuration
///
/// # Returns
///
/// A vector of (zone_id, face_count) pairs.
///
/// # Errors
///
/// Returns an error if any grow operation fails.
pub fn grow_multiple_regions(
    mesh: &IndexedMesh,
    adjacency: &FaceAdjacency,
    zone_map: &mut ZoneMap,
    seed_groups: &[Vec<usize>],
    config: &GrowConfig,
) -> ZoneResult<Vec<(u32, usize)>> {
    let mut results = Vec::new();

    for (i, seeds) in seed_groups.iter().enumerate() {
        let zone_id = (i + 1) as u32;
        let count = grow_region(mesh, adjacency, zone_map, seeds, zone_id, config)?;
        results.push((zone_id, count));
    }

    Ok(results)
}

/// Flood fill from a seed face, ignoring normals.
///
/// Assigns all connected faces to the same zone.
///
/// # Arguments
///
/// * `mesh` - The mesh to fill
/// * `adjacency` - Precomputed face adjacency
/// * `zone_map` - Zone map to update
/// * `seed` - Starting face index
/// * `zone_id` - Zone ID to assign
///
/// # Returns
///
/// The number of faces assigned.
///
/// # Errors
///
/// Returns an error if the seed is out of bounds.
pub fn flood_fill(
    mesh: &IndexedMesh,
    adjacency: &FaceAdjacency,
    zone_map: &mut ZoneMap,
    seed: usize,
    zone_id: u32,
) -> ZoneResult<usize> {
    grow_region(
        mesh,
        adjacency,
        zone_map,
        &[seed],
        zone_id,
        &GrowConfig::flood_fill(),
    )
}

/// Compute face normals for all faces.
fn compute_face_normals(mesh: &IndexedMesh) -> Vec<Vector3<f64>> {
    mesh.faces
        .iter()
        .map(|face| {
            let v0 = &mesh.vertices[face[0] as usize].position;
            let v1 = &mesh.vertices[face[1] as usize].position;
            let v2 = &mesh.vertices[face[2] as usize].position;

            let e1 = *v1 - *v0;
            let e2 = *v2 - *v0;
            let normal = e1.cross(&e2);

            normal.try_normalize(f64::EPSILON).unwrap_or(Vector3::z())
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn simple_strip() -> IndexedMesh {
        // 4 triangles in a strip
        let mut mesh = IndexedMesh::new();
        for i in 0..5 {
            mesh.vertices
                .push(Vertex::from_coords(i as f64, 0.0, 0.0));
            mesh.vertices
                .push(Vertex::from_coords(i as f64, 1.0, 0.0));
        }
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([2, 3, 1]);
        mesh.faces.push([2, 4, 3]);
        mesh.faces.push([4, 5, 3]);
        mesh
    }

    fn bent_strip() -> IndexedMesh {
        // 4 triangles, last two are rotated 90 degrees
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0));
        // Bent part
        mesh.vertices.push(Vertex::from_coords(2.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(2.0, 1.0, 1.0));

        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([2, 3, 1]);
        mesh.faces.push([2, 4, 3]);
        mesh.faces.push([4, 5, 3]);
        mesh
    }

    #[test]
    fn grow_config_default() {
        let config = GrowConfig::default();
        assert!(config.max_angle > 0.0);
        assert!(config.respect_boundaries);
    }

    #[test]
    fn grow_config_presets() {
        let flat = GrowConfig::flat();
        let smooth = GrowConfig::smooth();
        let flood = GrowConfig::flood_fill();

        assert!(flat.max_angle < smooth.max_angle);
        assert!(smooth.max_angle < flood.max_angle);
    }

    #[test]
    fn grow_simple_region() {
        let mesh = simple_strip();
        let adj = FaceAdjacency::from_mesh(&mesh);
        let mut zone_map = ZoneMap::new(mesh.faces.len());

        let count = grow_region(
            &mesh,
            &adj,
            &mut zone_map,
            &[0],
            1,
            &GrowConfig::flood_fill(),
        );

        assert!(count.is_ok());
        assert_eq!(count.expect("count"), 4);
    }

    #[test]
    fn grow_empty_mesh_fails() {
        let mesh = IndexedMesh::new();
        let adj = FaceAdjacency::from_mesh(&mesh);
        let mut zone_map = ZoneMap::new(0);

        let result = grow_region(&mesh, &adj, &mut zone_map, &[0], 1, &GrowConfig::default());

        assert!(result.is_err());
    }

    #[test]
    fn grow_no_seeds_fails() {
        let mesh = simple_strip();
        let adj = FaceAdjacency::from_mesh(&mesh);
        let mut zone_map = ZoneMap::new(mesh.faces.len());

        let result = grow_region(&mesh, &adj, &mut zone_map, &[], 1, &GrowConfig::default());

        assert!(result.is_err());
    }

    #[test]
    fn grow_invalid_seed_fails() {
        let mesh = simple_strip();
        let adj = FaceAdjacency::from_mesh(&mesh);
        let mut zone_map = ZoneMap::new(mesh.faces.len());

        let result = grow_region(&mesh, &adj, &mut zone_map, &[100], 1, &GrowConfig::default());

        assert!(result.is_err());
    }

    #[test]
    fn grow_with_angle_limit() {
        let mesh = bent_strip();
        let adj = FaceAdjacency::from_mesh(&mesh);
        let mut zone_map = ZoneMap::new(mesh.faces.len());

        // With small angle tolerance, shouldn't grow across the bend
        let count = grow_region(&mesh, &adj, &mut zone_map, &[0], 1, &GrowConfig::flat());

        assert!(count.is_ok());
        // Should only grow within the flat portion
        let count = count.expect("count");
        assert!(count < 4);
    }

    #[test]
    fn grow_multiple_regions_test() {
        let mesh = simple_strip();
        let adj = FaceAdjacency::from_mesh(&mesh);
        let mut zone_map = ZoneMap::new(mesh.faces.len());

        let seed_groups = vec![vec![0], vec![3]];
        let config = GrowConfig {
            respect_boundaries: false,
            ..GrowConfig::default()
        };

        let results = grow_multiple_regions(&mesh, &adj, &mut zone_map, &seed_groups, &config);

        assert!(results.is_ok());
        let results = results.expect("results");
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn flood_fill_test() {
        let mesh = simple_strip();
        let adj = FaceAdjacency::from_mesh(&mesh);
        let mut zone_map = ZoneMap::new(mesh.faces.len());

        let count = flood_fill(&mesh, &adj, &mut zone_map, 0, 1);

        assert!(count.is_ok());
        assert_eq!(count.expect("count"), 4);
    }

    #[test]
    fn respects_boundaries() {
        let mesh = simple_strip();
        let adj = FaceAdjacency::from_mesh(&mesh);
        let mut zone_map = ZoneMap::new(mesh.faces.len());

        // Assign face 2 to zone 99
        zone_map.set(2, 99).expect("should set");

        // Grow from face 0 with boundary respect (use default which respects boundaries)
        // Need to use high angle tolerance so normal angle doesn't block, but still respect_boundaries
        let config = GrowConfig::default().with_max_angle(std::f64::consts::PI);
        let count = grow_region(&mesh, &adj, &mut zone_map, &[0], 1, &config);

        assert!(count.is_ok());
        // Should stop at the boundary (faces 0, 1, and 3 are reachable but face 2 is blocked)
        // With adjacency: 0-1-2-3, blocking 2 means we can reach 0 and 1 only
        let count = count.expect("count");
        assert!(count < 4);
    }
}
