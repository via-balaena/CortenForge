//! Region selection criteria.
//!
//! The [`RegionSelector`] enum provides various spatial and topological
//! criteria for selecting vertices and faces from a mesh.

use hashbrown::{HashMap, HashSet};
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};
use std::collections::VecDeque;

/// Selector for defining regions based on spatial or topological criteria.
///
/// Selectors can be combined using boolean operations (AND, OR, NOT).
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex, Point3};
/// use mesh_region::RegionSelector;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
/// mesh.vertices.push(Vertex::new(Point3::new(10.0, 0.0, 0.0)));
/// mesh.vertices.push(Vertex::new(Point3::new(5.0, 10.0, 5.0)));
/// mesh.faces.push([0, 1, 2]);
///
/// // Select vertices in upper half
/// let selector = RegionSelector::half_space(
///     Point3::new(0.0, 0.0, 2.5),
///     nalgebra::Vector3::new(0.0, 0.0, 1.0),
/// );
///
/// let vertices = selector.select_vertices(&mesh);
/// assert_eq!(vertices.len(), 1);
/// ```
#[derive(Debug, Clone)]
pub enum RegionSelector {
    /// Select vertices/faces within an axis-aligned bounding box.
    Bounds {
        /// Minimum corner.
        min: Point3<f64>,
        /// Maximum corner.
        max: Point3<f64>,
    },

    /// Select vertices/faces within a sphere.
    Sphere {
        /// Center point.
        center: Point3<f64>,
        /// Radius.
        radius: f64,
    },

    /// Select vertices/faces within a cylinder.
    Cylinder {
        /// Start point of axis.
        axis_start: Point3<f64>,
        /// End point of axis.
        axis_end: Point3<f64>,
        /// Cylinder radius.
        radius: f64,
    },

    /// Select vertices/faces within a distance of a plane.
    Plane {
        /// Point on the plane.
        point: Point3<f64>,
        /// Normal vector (will be normalized).
        normal: Vector3<f64>,
        /// Maximum distance from plane.
        tolerance: f64,
    },

    /// Select vertices/faces above a plane (in normal direction).
    HalfSpace {
        /// Point on the plane.
        point: Point3<f64>,
        /// Normal vector (will be normalized).
        normal: Vector3<f64>,
    },

    /// Select by explicit vertex indices.
    Vertices(HashSet<u32>),

    /// Select by explicit face indices.
    Faces(HashSet<u32>),

    /// Combine selectors with AND (intersection).
    And(Box<RegionSelector>, Box<RegionSelector>),

    /// Combine selectors with OR (union).
    Or(Box<RegionSelector>, Box<RegionSelector>),

    /// Negate a selector (complement).
    Not(Box<RegionSelector>),

    /// Flood-fill from seed faces, expanding based on criteria.
    FloodFill {
        /// Seed face indices to start the flood-fill from.
        seeds: HashSet<u32>,
        /// Stopping criteria for the flood-fill.
        criteria: FloodFillCriteria,
    },
}

/// Criteria for stopping flood-fill expansion.
///
/// Flood-fill expands from seed faces to adjacent faces until one of these
/// criteria is met.
#[derive(Debug, Clone)]
pub struct FloodFillCriteria {
    /// Maximum angle (in radians) between adjacent face normals.
    /// Flood-fill stops at edges where the dihedral angle exceeds this.
    /// Default: π/6 (30 degrees).
    pub max_angle: f64,

    /// Maximum distance from the seed faces' centroid.
    /// None means no distance limit.
    pub max_distance: Option<f64>,

    /// Maximum number of faces to include.
    /// None means no limit.
    pub max_faces: Option<usize>,

    /// Whether to stop at boundary edges (edges with only one adjacent face).
    /// Default: true.
    pub stop_at_boundary: bool,
}

impl Default for FloodFillCriteria {
    fn default() -> Self {
        Self {
            max_angle: std::f64::consts::PI / 6.0, // 30 degrees
            max_distance: None,
            max_faces: None,
            stop_at_boundary: true,
        }
    }
}

impl FloodFillCriteria {
    /// Create criteria that stops at sharp edges (angle threshold).
    #[must_use]
    pub fn angle_threshold(radians: f64) -> Self {
        Self {
            max_angle: radians,
            ..Default::default()
        }
    }

    /// Create criteria with a distance limit from seed.
    #[must_use]
    pub fn with_max_distance(mut self, distance: f64) -> Self {
        self.max_distance = Some(distance);
        self
    }

    /// Create criteria with a face count limit.
    #[must_use]
    pub fn with_max_faces(mut self, count: usize) -> Self {
        self.max_faces = Some(count);
        self
    }

    /// Set whether to stop at mesh boundaries.
    #[must_use]
    pub fn stop_at_boundary(mut self, stop: bool) -> Self {
        self.stop_at_boundary = stop;
        self
    }

    /// Create very permissive criteria (expand everywhere possible).
    #[must_use]
    pub fn permissive() -> Self {
        Self {
            max_angle: std::f64::consts::PI, // 180 degrees
            max_distance: None,
            max_faces: None,
            stop_at_boundary: false,
        }
    }

    /// Create criteria for selecting smooth regions (strict angle threshold).
    #[must_use]
    pub fn smooth_regions() -> Self {
        Self {
            max_angle: std::f64::consts::PI / 12.0, // 15 degrees
            ..Default::default()
        }
    }
}

impl RegionSelector {
    /// Create a bounding box selector.
    #[must_use]
    pub fn bounds(min: Point3<f64>, max: Point3<f64>) -> Self {
        Self::Bounds { min, max }
    }

    /// Create a sphere selector.
    #[must_use]
    pub fn sphere(center: Point3<f64>, radius: f64) -> Self {
        Self::Sphere { center, radius }
    }

    /// Create a cylinder selector.
    #[must_use]
    pub fn cylinder(axis_start: Point3<f64>, axis_end: Point3<f64>, radius: f64) -> Self {
        Self::Cylinder {
            axis_start,
            axis_end,
            radius,
        }
    }

    /// Create a plane selector (vertices within tolerance of plane).
    #[must_use]
    pub fn plane(point: Point3<f64>, normal: Vector3<f64>, tolerance: f64) -> Self {
        Self::Plane {
            point,
            normal: normal.normalize(),
            tolerance,
        }
    }

    /// Create a half-space selector (vertices above plane).
    #[must_use]
    pub fn half_space(point: Point3<f64>, normal: Vector3<f64>) -> Self {
        Self::HalfSpace {
            point,
            normal: normal.normalize(),
        }
    }

    /// Create a selector from explicit vertex indices.
    #[must_use]
    pub fn vertices(indices: impl IntoIterator<Item = u32>) -> Self {
        Self::Vertices(indices.into_iter().collect())
    }

    /// Create a selector from explicit face indices.
    #[must_use]
    pub fn faces(indices: impl IntoIterator<Item = u32>) -> Self {
        Self::Faces(indices.into_iter().collect())
    }

    /// Combine with another selector using AND.
    #[must_use]
    pub fn and(self, other: Self) -> Self {
        Self::And(Box::new(self), Box::new(other))
    }

    /// Combine with another selector using OR.
    #[must_use]
    pub fn or(self, other: Self) -> Self {
        Self::Or(Box::new(self), Box::new(other))
    }

    /// Negate this selector.
    #[must_use]
    #[allow(clippy::should_implement_trait)] // This is a builder method, not std::ops::Not
    pub fn not(self) -> Self {
        Self::Not(Box::new(self))
    }

    /// Create a flood-fill selector starting from a single seed face.
    #[must_use]
    pub fn flood_fill(seed_face: u32, criteria: FloodFillCriteria) -> Self {
        let mut seeds = HashSet::new();
        seeds.insert(seed_face);
        Self::FloodFill { seeds, criteria }
    }

    /// Create a flood-fill selector starting from multiple seed faces.
    #[must_use]
    pub fn flood_fill_multi(
        seed_faces: impl IntoIterator<Item = u32>,
        criteria: FloodFillCriteria,
    ) -> Self {
        Self::FloodFill {
            seeds: seed_faces.into_iter().collect(),
            criteria,
        }
    }

    /// Apply this selector to a mesh, returning (vertices, faces).
    #[must_use]
    pub fn select(&self, mesh: &IndexedMesh) -> (HashSet<u32>, HashSet<u32>) {
        let vertices = self.select_vertices(mesh);

        // Also select faces where all vertices are selected
        let faces: HashSet<u32> = mesh
            .faces
            .iter()
            .enumerate()
            .filter(|(_, face)| {
                vertices.contains(&face[0])
                    && vertices.contains(&face[1])
                    && vertices.contains(&face[2])
            })
            .filter_map(|(i, _)| u32::try_from(i).ok())
            .collect();

        (vertices, faces)
    }

    /// Select only vertices from this selector.
    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn select_vertices(&self, mesh: &IndexedMesh) -> HashSet<u32> {
        match self {
            Self::Bounds { min, max } => select_bounds(mesh, min, max),
            Self::Sphere { center, radius } => select_sphere(mesh, center, *radius),
            Self::Cylinder {
                axis_start,
                axis_end,
                radius,
            } => select_cylinder(mesh, axis_start, axis_end, *radius),
            Self::Plane {
                point,
                normal,
                tolerance,
            } => select_plane(mesh, point, normal, *tolerance),
            Self::HalfSpace { point, normal } => select_half_space(mesh, point, normal),
            Self::Vertices(indices) => indices.clone(),
            Self::Faces(face_indices) => select_faces_to_vertices(mesh, face_indices),
            Self::And(a, b) => {
                let va = a.select_vertices(mesh);
                let vb = b.select_vertices(mesh);
                va.intersection(&vb).copied().collect()
            }
            Self::Or(a, b) => {
                let va = a.select_vertices(mesh);
                let vb = b.select_vertices(mesh);
                va.union(&vb).copied().collect()
            }
            Self::Not(inner) => {
                let selected = inner.select_vertices(mesh);
                let vertex_count = u32::try_from(mesh.vertices.len()).unwrap_or(u32::MAX);
                (0..vertex_count)
                    .filter(|i| !selected.contains(i))
                    .collect()
            }
            Self::FloodFill { seeds, criteria } => {
                let faces = flood_fill_faces(mesh, seeds, criteria);
                let mut vertices = HashSet::new();
                for fi in faces {
                    if let Some(face) = mesh.faces.get(fi as usize) {
                        vertices.insert(face[0]);
                        vertices.insert(face[1]);
                        vertices.insert(face[2]);
                    }
                }
                vertices
            }
        }
    }

    /// Select faces using this selector (for flood-fill, this is the primary method).
    #[must_use]
    pub fn select_faces(&self, mesh: &IndexedMesh) -> HashSet<u32> {
        if let Self::FloodFill { seeds, criteria } = self {
            return flood_fill_faces(mesh, seeds, criteria);
        }

        // For other selectors, select faces where all vertices are selected
        let vertices = self.select_vertices(mesh);
        mesh.faces
            .iter()
            .enumerate()
            .filter(|(_, face)| {
                vertices.contains(&face[0])
                    && vertices.contains(&face[1])
                    && vertices.contains(&face[2])
            })
            .filter_map(|(i, _)| u32::try_from(i).ok())
            .collect()
    }
}

// Helper functions to keep select_vertices under the line limit

fn select_bounds(mesh: &IndexedMesh, min: &Point3<f64>, max: &Point3<f64>) -> HashSet<u32> {
    mesh.vertices
        .iter()
        .enumerate()
        .filter(|(_, v)| {
            v.position.x >= min.x
                && v.position.x <= max.x
                && v.position.y >= min.y
                && v.position.y <= max.y
                && v.position.z >= min.z
                && v.position.z <= max.z
        })
        .filter_map(|(i, _)| u32::try_from(i).ok())
        .collect()
}

fn select_sphere(mesh: &IndexedMesh, center: &Point3<f64>, radius: f64) -> HashSet<u32> {
    mesh.vertices
        .iter()
        .enumerate()
        .filter(|(_, v)| (v.position - center).norm() <= radius)
        .filter_map(|(i, _)| u32::try_from(i).ok())
        .collect()
}

fn select_cylinder(
    mesh: &IndexedMesh,
    axis_start: &Point3<f64>,
    axis_end: &Point3<f64>,
    radius: f64,
) -> HashSet<u32> {
    let axis = axis_end - axis_start;
    let axis_len_sq = axis.norm_squared();
    if axis_len_sq < 1e-10 {
        return HashSet::new();
    }

    mesh.vertices
        .iter()
        .enumerate()
        .filter(|(_, v)| {
            let to_point = v.position - axis_start;
            let t = to_point.dot(&axis) / axis_len_sq;
            if !(0.0..=1.0).contains(&t) {
                return false;
            }
            let projection = axis_start + axis * t;
            (v.position - projection).norm() <= radius
        })
        .filter_map(|(i, _)| u32::try_from(i).ok())
        .collect()
}

fn select_plane(
    mesh: &IndexedMesh,
    point: &Point3<f64>,
    normal: &Vector3<f64>,
    tolerance: f64,
) -> HashSet<u32> {
    mesh.vertices
        .iter()
        .enumerate()
        .filter(|(_, v)| (v.position - point).dot(normal).abs() <= tolerance)
        .filter_map(|(i, _)| u32::try_from(i).ok())
        .collect()
}

fn select_half_space(
    mesh: &IndexedMesh,
    point: &Point3<f64>,
    normal: &Vector3<f64>,
) -> HashSet<u32> {
    mesh.vertices
        .iter()
        .enumerate()
        .filter(|(_, v)| (v.position - point).dot(normal) >= 0.0)
        .filter_map(|(i, _)| u32::try_from(i).ok())
        .collect()
}

fn select_faces_to_vertices(mesh: &IndexedMesh, face_indices: &HashSet<u32>) -> HashSet<u32> {
    let mut vertices = HashSet::new();
    for &fi in face_indices {
        if let Some(face) = mesh.faces.get(fi as usize) {
            vertices.insert(face[0]);
            vertices.insert(face[1]);
            vertices.insert(face[2]);
        }
    }
    vertices
}

/// Perform flood-fill on mesh faces starting from seed faces.
#[allow(clippy::too_many_lines)]
fn flood_fill_faces(
    mesh: &IndexedMesh,
    seeds: &HashSet<u32>,
    criteria: &FloodFillCriteria,
) -> HashSet<u32> {
    if seeds.is_empty() || mesh.faces.is_empty() {
        return HashSet::new();
    }

    // Build edge-to-face adjacency
    let edge_to_faces = build_edge_to_faces(&mesh.faces);

    // Precompute face normals and centroids
    let face_normals: Vec<Option<Vector3<f64>>> = mesh
        .faces
        .iter()
        .map(|face| {
            let v0 = mesh.vertices.get(face[0] as usize)?.position;
            let v1 = mesh.vertices.get(face[1] as usize)?.position;
            let v2 = mesh.vertices.get(face[2] as usize)?.position;
            let e1 = v1 - v0;
            let e2 = v2 - v0;
            let normal = e1.cross(&e2);
            let len = normal.norm();
            if len > 1e-10 {
                Some(normal / len)
            } else {
                None
            }
        })
        .collect();

    let face_centroids: Vec<Option<Point3<f64>>> = mesh
        .faces
        .iter()
        .map(|face| {
            let v0 = mesh.vertices.get(face[0] as usize)?.position;
            let v1 = mesh.vertices.get(face[1] as usize)?.position;
            let v2 = mesh.vertices.get(face[2] as usize)?.position;
            Some(Point3::from((v0.coords + v1.coords + v2.coords) / 3.0))
        })
        .collect();

    // Compute seed centroid for distance limit
    let seed_centroid: Option<Point3<f64>> = if criteria.max_distance.is_some() {
        let mut sum = Vector3::zeros();
        let mut count = 0;
        for &seed in seeds {
            if let Some(Some(centroid)) = face_centroids.get(seed as usize) {
                sum += centroid.coords;
                count += 1;
            }
        }
        if count > 0 {
            Some(Point3::from(sum / f64::from(count)))
        } else {
            None
        }
    } else {
        None
    };

    // Initialize flood-fill
    let mut selected: HashSet<u32> = HashSet::new();
    let mut queue: VecDeque<u32> = VecDeque::new();

    // Add seeds to the queue
    for &seed in seeds {
        if (seed as usize) < mesh.faces.len() {
            selected.insert(seed);
            queue.push_back(seed);
        }
    }

    // Flood-fill BFS
    while let Some(current_face) = queue.pop_front() {
        // Check face limit
        if criteria.max_faces.is_some_and(|max| selected.len() >= max) {
            break;
        }

        let current_normal = match face_normals.get(current_face as usize) {
            Some(Some(n)) => *n,
            _ => continue,
        };

        // Get edges of current face
        let Some(face) = mesh.faces.get(current_face as usize) else {
            continue;
        };
        let edges = [
            normalize_edge(face[0], face[1]),
            normalize_edge(face[1], face[2]),
            normalize_edge(face[2], face[0]),
        ];

        // Check each adjacent face
        for edge in &edges {
            let Some(adjacent_faces) = edge_to_faces.get(edge) else {
                continue;
            };

            // Check if this is a boundary edge
            if criteria.stop_at_boundary && adjacent_faces.len() == 1 {
                continue;
            }

            for &neighbor in adjacent_faces {
                if neighbor == current_face || selected.contains(&neighbor) {
                    continue;
                }

                // Check face limit before adding
                if criteria.max_faces.is_some_and(|max| selected.len() >= max) {
                    break;
                }

                // Get neighbor normal
                let neighbor_normal = match face_normals.get(neighbor as usize) {
                    Some(Some(n)) => *n,
                    _ => continue,
                };

                // Check angle criterion
                let dot = current_normal.dot(&neighbor_normal).clamp(-1.0, 1.0);
                let angle = dot.acos();
                if angle > criteria.max_angle {
                    continue;
                }

                // Check distance criterion
                if !check_distance_criterion(
                    &face_centroids,
                    neighbor,
                    seed_centroid,
                    criteria.max_distance,
                ) {
                    continue;
                }

                // All criteria passed, add to selection
                selected.insert(neighbor);
                queue.push_back(neighbor);
            }
        }
    }

    selected
}

/// Check if a neighbor face passes the distance criterion.
fn check_distance_criterion(
    face_centroids: &[Option<Point3<f64>>],
    neighbor: u32,
    seed_centroid: Option<Point3<f64>>,
    max_distance: Option<f64>,
) -> bool {
    let Some(max_dist) = max_distance else {
        return true;
    };
    let Some(seed_center) = seed_centroid else {
        return true;
    };
    let Some(Some(neighbor_centroid)) = face_centroids.get(neighbor as usize) else {
        return true;
    };

    let dist = (neighbor_centroid - seed_center).norm();
    dist <= max_dist
}

/// Build a map from edges to the faces that contain them.
fn build_edge_to_faces(faces: &[[u32; 3]]) -> HashMap<(u32, u32), Vec<u32>> {
    let mut edge_to_faces: HashMap<(u32, u32), Vec<u32>> = HashMap::new();

    for (face_idx, face) in faces.iter().enumerate() {
        let edges = [
            normalize_edge(face[0], face[1]),
            normalize_edge(face[1], face[2]),
            normalize_edge(face[2], face[0]),
        ];

        let face_idx = u32::try_from(face_idx).unwrap_or(u32::MAX);
        for edge in edges {
            edge_to_faces.entry(edge).or_default().push(face_idx);
        }
    }

    edge_to_faces
}

/// Normalize an edge so the smaller vertex index comes first.
fn normalize_edge(v0: u32, v1: u32) -> (u32, u32) {
    if v0 < v1 { (v0, v1) } else { (v1, v0) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn create_test_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        // 8 vertices of a 10x10x10 cube
        mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(10.0, 0.0, 0.0)));
        mesh.vertices
            .push(Vertex::new(Point3::new(10.0, 10.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(0.0, 10.0, 0.0)));
        mesh.vertices.push(Vertex::new(Point3::new(0.0, 0.0, 10.0)));
        mesh.vertices
            .push(Vertex::new(Point3::new(10.0, 0.0, 10.0)));
        mesh.vertices
            .push(Vertex::new(Point3::new(10.0, 10.0, 10.0)));
        mesh.vertices
            .push(Vertex::new(Point3::new(0.0, 10.0, 10.0)));

        // 12 triangular faces (2 per cube face)
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        mesh.faces.push([4, 5, 6]);
        mesh.faces.push([4, 6, 7]);
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        mesh.faces.push([0, 4, 7]);
        mesh.faces.push([0, 7, 3]);
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);

        mesh
    }

    #[test]
    fn test_bounds_selector() {
        let mesh = create_test_cube();

        // Select bottom half
        let selector =
            RegionSelector::bounds(Point3::new(-1.0, -1.0, -1.0), Point3::new(11.0, 11.0, 5.0));

        let (vertices, _) = selector.select(&mesh);
        assert_eq!(vertices.len(), 4); // Bottom 4 vertices
    }

    #[test]
    fn test_sphere_selector() {
        let mesh = create_test_cube();

        // Select around center
        let selector = RegionSelector::sphere(Point3::new(5.0, 5.0, 5.0), 10.0);

        let (vertices, _) = selector.select(&mesh);
        assert_eq!(vertices.len(), 8); // All vertices within 10mm of center
    }

    #[test]
    fn test_half_space_selector() {
        let mesh = create_test_cube();

        // Select top half (z > 5)
        let selector =
            RegionSelector::half_space(Point3::new(0.0, 0.0, 5.0), Vector3::new(0.0, 0.0, 1.0));

        let (vertices, _) = selector.select(&mesh);
        assert_eq!(vertices.len(), 4); // Top 4 vertices
    }

    #[test]
    fn test_and_selector() {
        let mesh = create_test_cube();

        // Select top half AND x > 5
        let selector = RegionSelector::half_space(Point3::new(0.0, 0.0, 5.0), Vector3::z()).and(
            RegionSelector::half_space(Point3::new(5.0, 0.0, 0.0), Vector3::x()),
        );

        let (vertices, _) = selector.select(&mesh);
        assert_eq!(vertices.len(), 2); // Top-right vertices (5, 6)
    }

    #[test]
    fn test_or_selector() {
        let mesh = create_test_cube();

        // Select vertex 0 OR vertex 7
        let selector = RegionSelector::vertices([0]).or(RegionSelector::vertices([7]));

        let (vertices, _) = selector.select(&mesh);
        assert_eq!(vertices.len(), 2);
    }

    #[test]
    fn test_not_selector() {
        let mesh = create_test_cube();

        // Select all except vertex 0
        let selector = RegionSelector::vertices([0]).not();

        let (vertices, _) = selector.select(&mesh);
        assert_eq!(vertices.len(), 7);
        assert!(!vertices.contains(&0));
    }

    #[test]
    fn test_flood_fill_permissive() {
        let mesh = create_test_cube();

        let selector = RegionSelector::flood_fill(0, FloodFillCriteria::permissive());
        let faces = selector.select_faces(&mesh);

        // With permissive criteria, should expand to all connected faces
        assert_eq!(faces.len(), 12);
    }

    #[test]
    fn test_flood_fill_angle_limited() {
        let mesh = create_test_cube();

        // Strict angle limit (10 degrees)
        let selector = RegionSelector::flood_fill(
            0,
            FloodFillCriteria::angle_threshold(std::f64::consts::PI / 18.0),
        );
        let faces = selector.select_faces(&mesh);

        // Should only get coplanar faces (bottom of cube)
        assert!(faces.len() <= 2);
        assert!(faces.contains(&0));
    }

    #[test]
    fn test_flood_fill_max_faces() {
        let mesh = create_test_cube();

        let criteria = FloodFillCriteria::permissive().with_max_faces(3);
        let selector = RegionSelector::flood_fill(0, criteria);
        let faces = selector.select_faces(&mesh);

        assert!(faces.len() <= 3);
    }

    #[test]
    fn test_cylinder_selector() {
        let mesh = create_test_cube();

        // Cylinder along Z axis at center
        let selector = RegionSelector::cylinder(
            Point3::new(5.0, 5.0, 0.0),
            Point3::new(5.0, 5.0, 10.0),
            10.0,
        );

        let (vertices, _) = selector.select(&mesh);
        assert!(!vertices.is_empty());
    }

    #[test]
    fn test_plane_selector() {
        let mesh = create_test_cube();

        // Select vertices near z=0 plane
        let selector = RegionSelector::plane(Point3::new(0.0, 0.0, 0.0), Vector3::z(), 0.1);

        let (vertices, _) = selector.select(&mesh);
        assert_eq!(vertices.len(), 4); // Bottom 4 vertices
    }
}
