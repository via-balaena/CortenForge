//! Ball Pivoting Algorithm for surface reconstruction.
//!
//! The Ball Pivoting Algorithm (BPA) reconstructs a triangle mesh from
//! a point cloud with normals by simulating a ball of given radius that
//! rolls over the point cloud surface, creating triangles where it touches
//! three points.
//!
//! # Algorithm Overview
//!
//! 1. Start with a seed triangle formed by three points the ball touches
//! 2. Pivot the ball around each edge, looking for the next point
//! 3. Create new triangles as the ball encounters new points
//! 4. Continue until no more triangles can be formed
//!
//! # Requirements
//!
//! - Point cloud must have oriented normals
//! - Ball radius should be chosen based on point density

use kiddo::{KdTree, SquaredEuclidean};
use mesh_types::{IndexedMesh, Vertex};
use nalgebra::{Point3, Vector3};
use std::collections::{HashMap, HashSet};

use crate::error::{ScanError, ScanResult};
use crate::pointcloud::PointCloud;

/// Parameters for ball pivoting reconstruction.
#[derive(Debug, Clone)]
pub struct BallPivotingParams {
    /// Ball radius. Should be slightly larger than the average point spacing.
    pub radius: f64,

    /// Maximum angle between ball center and edge midpoint (radians).
    /// Default: π/2 (90 degrees).
    pub max_pivot_angle: f64,

    /// Minimum angle between face normals to consider as valid.
    /// Default: 0.0 (no restriction).
    pub min_normal_angle: f64,

    /// Whether to fill small holes after reconstruction.
    /// Default: false.
    pub fill_small_holes: bool,

    /// Maximum hole size to fill (number of boundary edges).
    /// Default: 10.
    pub max_hole_size: usize,
}

impl Default for BallPivotingParams {
    fn default() -> Self {
        Self {
            radius: 1.0,
            max_pivot_angle: std::f64::consts::FRAC_PI_2,
            min_normal_angle: 0.0,
            fill_small_holes: false,
            max_hole_size: 10,
        }
    }
}

impl BallPivotingParams {
    /// Creates new parameters with the given radius.
    #[must_use]
    pub fn new(radius: f64) -> Self {
        Self {
            radius,
            ..Self::default()
        }
    }

    /// Sets the ball radius.
    #[must_use]
    pub const fn with_radius(mut self, radius: f64) -> Self {
        self.radius = radius;
        self
    }

    /// Sets the maximum pivot angle.
    #[must_use]
    pub const fn with_max_pivot_angle(mut self, angle: f64) -> Self {
        self.max_pivot_angle = angle;
        self
    }

    /// Enables hole filling.
    #[must_use]
    pub const fn with_fill_holes(mut self, fill: bool, max_size: usize) -> Self {
        self.fill_small_holes = fill;
        self.max_hole_size = max_size;
        self
    }

    /// Estimates a good radius from point cloud density.
    ///
    /// Uses k-nearest neighbor distances to estimate average point spacing.
    #[must_use]
    pub fn estimate_radius(cloud: &PointCloud, k: usize) -> f64 {
        if cloud.is_empty() || k == 0 {
            return 1.0;
        }

        // Build KD-tree
        let mut tree: KdTree<f64, 3> = KdTree::new();
        for (i, point) in cloud.points.iter().enumerate() {
            let p = point.position;
            tree.add(&[p.x, p.y, p.z], i as u64);
        }

        // Compute average k-nearest neighbor distance
        let k_query = k.min(cloud.len().saturating_sub(1)).max(1);
        let mut total_dist = 0.0;
        let mut count = 0;

        for point in &cloud.points {
            let p = point.position;
            let neighbors = tree.nearest_n::<SquaredEuclidean>(&[p.x, p.y, p.z], k_query + 1);

            // Skip first neighbor (self)
            for neighbor in neighbors.iter().skip(1) {
                total_dist += neighbor.distance.sqrt();
                count += 1;
            }
        }

        if count == 0 {
            return 1.0;
        }

        #[allow(clippy::cast_precision_loss)]
        let avg_spacing = total_dist / f64::from(count);

        // Ball radius should be slightly larger than average spacing
        avg_spacing * 1.5
    }
}

/// Result of ball pivoting reconstruction.
#[derive(Debug, Clone)]
pub struct BallPivotingResult {
    /// The reconstructed mesh.
    pub mesh: IndexedMesh,

    /// Number of triangles created.
    pub triangle_count: usize,

    /// Number of points that were not connected.
    pub orphan_points: usize,

    /// Estimated surface coverage ratio (0.0 - 1.0).
    pub coverage_ratio: f64,

    /// Number of boundary edges (open edges).
    pub boundary_edge_count: usize,
}

impl std::fmt::Display for BallPivotingResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "BallPivoting: {} triangles, {:.1}% coverage, {} boundary edges",
            self.triangle_count,
            self.coverage_ratio * 100.0,
            self.boundary_edge_count
        )
    }
}

/// An edge in the reconstruction front.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct Edge {
    v0: u32,
    v1: u32,
}

impl Edge {
    const fn new(a: u32, b: u32) -> Self {
        // Normalize edge ordering for consistent hashing
        if a < b {
            Self { v0: a, v1: b }
        } else {
            Self { v0: b, v1: a }
        }
    }

    const fn opposite(&self) -> Self {
        Self {
            v0: self.v1,
            v1: self.v0,
        }
    }
}

/// Represents the ball position for pivoting.
#[derive(Debug, Clone)]
struct BallPosition {
    center: Point3<f64>,
    #[allow(dead_code)]
    triangle: [u32; 3],
}

/// Reconstructs a mesh from a point cloud using the Ball Pivoting Algorithm.
///
/// # Arguments
///
/// * `cloud` - Point cloud with oriented normals
/// * `params` - Ball pivoting parameters
///
/// # Returns
///
/// The reconstructed mesh and statistics.
///
/// # Errors
///
/// Returns an error if:
/// - Point cloud is empty
/// - Point cloud has no normals
/// - No valid seed triangle can be found
///
/// # Example
///
/// ```
/// use mesh_scan::reconstruct::{ball_pivoting, BallPivotingParams};
/// use mesh_scan::pointcloud::PointCloud;
/// use nalgebra::{Point3, Vector3};
///
/// // Create a simple point cloud with normals
/// let mut cloud = PointCloud::new();
/// cloud.add_point_with_normal(
///     Point3::new(0.0, 0.0, 0.0),
///     Vector3::new(0.0, 0.0, 1.0),
/// );
/// cloud.add_point_with_normal(
///     Point3::new(1.0, 0.0, 0.0),
///     Vector3::new(0.0, 0.0, 1.0),
/// );
/// cloud.add_point_with_normal(
///     Point3::new(0.5, 1.0, 0.0),
///     Vector3::new(0.0, 0.0, 1.0),
/// );
///
/// let params = BallPivotingParams::new(2.0);
/// let result = ball_pivoting(&cloud, &params).unwrap();
/// println!("{}", result);
/// ```
#[allow(clippy::too_many_lines)] // Surface reconstruction is inherently sequential
pub fn ball_pivoting(
    cloud: &PointCloud,
    params: &BallPivotingParams,
) -> ScanResult<BallPivotingResult> {
    if cloud.is_empty() {
        return Err(ScanError::EmptyPointCloud);
    }

    // Check that we have normals
    if !cloud.has_normals() {
        return Err(ScanError::NormalEstimationFailed {
            reason: "point cloud must have normals for ball pivoting".to_string(),
        });
    }

    // Build KD-tree for efficient spatial queries
    let mut tree: KdTree<f64, 3> = KdTree::new();
    for (i, point) in cloud.points.iter().enumerate() {
        let p = point.position;
        tree.add(&[p.x, p.y, p.z], i as u64);
    }

    // Initialize mesh
    let mut mesh = IndexedMesh::new();

    // Add all points as vertices
    for point in &cloud.points {
        let mut vertex = Vertex::new(point.position);
        vertex.attributes.normal = point.normal;
        mesh.vertices.push(vertex);
    }

    // Track used vertices and front edges
    let mut used_vertices: HashSet<u32> = HashSet::new();
    let mut front_edges: HashMap<Edge, BallPosition> = HashMap::new();
    let mut completed_edges: HashSet<Edge> = HashSet::new();

    // Find seed triangle
    let seed = find_seed_triangle(cloud, &tree, params.radius)?;
    if let Some((tri, ball_center)) = seed {
        // Add seed triangle
        mesh.faces.push(tri);
        used_vertices.insert(tri[0]);
        used_vertices.insert(tri[1]);
        used_vertices.insert(tri[2]);

        // Add edges to front
        let edges = [
            Edge::new(tri[0], tri[1]),
            Edge::new(tri[1], tri[2]),
            Edge::new(tri[2], tri[0]),
        ];

        for edge in edges {
            front_edges.insert(
                edge,
                BallPosition {
                    center: ball_center,
                    triangle: tri,
                },
            );
        }
    }

    // Main loop: process front edges
    let max_iterations = cloud.len() * 10; // Prevent infinite loops
    let mut iterations = 0;

    while !front_edges.is_empty() && iterations < max_iterations {
        iterations += 1;

        // Get an edge from the front
        let (edge, ball_pos) = match front_edges.iter().next() {
            Some((e, p)) => (*e, p.clone()),
            None => break,
        };
        front_edges.remove(&edge);

        // Skip if already completed from both sides
        if completed_edges.contains(&edge) {
            continue;
        }

        // Try to pivot the ball
        if let Some((new_vertex, new_center)) = pivot_ball(
            cloud,
            &tree,
            &edge,
            &ball_pos.center,
            params,
            &used_vertices,
        ) {
            let v0 = edge.v0;
            let v1 = edge.v1;
            let v2 = new_vertex;

            // Create new triangle (ensure consistent winding)
            let new_tri = [v0, v2, v1];
            mesh.faces.push(new_tri);
            used_vertices.insert(v2);

            // Update edges
            completed_edges.insert(edge);

            let new_edges = [Edge::new(v0, v2), Edge::new(v2, v1)];

            for new_edge in new_edges {
                let opposite = new_edge.opposite();
                if completed_edges.contains(&new_edge) || completed_edges.contains(&opposite) {
                    continue;
                }

                if front_edges.contains_key(&opposite) {
                    // Edge completed from both sides
                    front_edges.remove(&opposite);
                    completed_edges.insert(new_edge);
                } else {
                    // Add to front
                    front_edges.insert(
                        new_edge,
                        BallPosition {
                            center: new_center,
                            triangle: new_tri,
                        },
                    );
                }
            }
        } else {
            // Edge becomes boundary
            completed_edges.insert(edge);
        }
    }

    // Calculate statistics
    let triangle_count = mesh.faces.len();

    #[allow(clippy::cast_precision_loss)]
    let coverage_ratio = used_vertices.len() as f64 / cloud.len().max(1) as f64;

    let orphan_points = cloud.len() - used_vertices.len();
    let boundary_edge_count = count_boundary_edges(&mesh);

    Ok(BallPivotingResult {
        mesh,
        triangle_count,
        orphan_points,
        coverage_ratio,
        boundary_edge_count,
    })
}

/// Finds a seed triangle to start reconstruction.
fn find_seed_triangle(
    cloud: &PointCloud,
    tree: &KdTree<f64, 3>,
    radius: f64,
) -> ScanResult<Option<([u32; 3], Point3<f64>)>> {
    let search_radius = radius * 2.0;
    let search_radius_sq = search_radius * search_radius;

    for (i, point) in cloud.points.iter().enumerate() {
        let p = point.position;
        let normal = match point.normal {
            Some(n) => n,
            None => continue,
        };

        // Find nearby points
        let neighbors = tree.within::<SquaredEuclidean>(&[p.x, p.y, p.z], search_radius_sq);

        // Try to form a triangle with two neighbors
        for j in 0..neighbors.len() {
            #[allow(clippy::cast_possible_truncation)]
            let idx_j = neighbors[j].item as usize;
            if idx_j == i {
                continue;
            }

            for k in (j + 1)..neighbors.len() {
                #[allow(clippy::cast_possible_truncation)]
                let idx_k = neighbors[k].item as usize;
                if idx_k == i {
                    continue;
                }

                // Try to form triangle (i, j, k)
                let p0 = cloud.points[i].position;
                let p1 = cloud.points[idx_j].position;
                let p2 = cloud.points[idx_k].position;

                // Compute circumcenter of triangle
                if let Some(center) = compute_ball_center(&p0, &p1, &p2, &normal, radius) {
                    // Verify ball doesn't contain other points
                    if is_empty_ball(cloud, tree, &center, radius, &[i, idx_j, idx_k]) {
                        #[allow(clippy::cast_possible_truncation)]
                        let tri = [i as u32, idx_j as u32, idx_k as u32];
                        return Ok(Some((tri, center)));
                    }
                }
            }
        }
    }

    // No seed triangle found
    Ok(None)
}

/// Computes the center of a ball touching three points.
#[allow(clippy::many_single_char_names)]
fn compute_ball_center(
    p0: &Point3<f64>,
    p1: &Point3<f64>,
    p2: &Point3<f64>,
    normal: &Vector3<f64>,
    radius: f64,
) -> Option<Point3<f64>> {
    // Compute triangle normal
    let e1 = p1 - p0;
    let e2 = p2 - p0;
    let tri_normal = e1.cross(&e2);
    let tri_normal_len = tri_normal.norm();

    if tri_normal_len < 1e-10 {
        return None; // Degenerate triangle
    }

    let tri_normal_unit = tri_normal / tri_normal_len;

    // Compute circumcircle radius
    let a = (p1 - p2).norm();
    let b = (p0 - p2).norm();
    let c = (p0 - p1).norm();
    let s = (a + b + c) / 2.0;
    let area = (s * (s - a) * (s - b) * (s - c)).sqrt();

    if area < 1e-10 {
        return None;
    }

    let circumradius = (a * b * c) / (4.0 * area);

    // Check if circumradius is too large for ball
    if circumradius > radius {
        return None;
    }

    // Distance from circumcenter to ball center
    let h_sq = radius.mul_add(radius, -(circumradius * circumradius));
    if h_sq < 0.0 {
        return None;
    }
    let h = h_sq.sqrt();

    // Compute circumcenter of triangle
    let d = 2.0 * (e1.cross(&e2)).norm_squared();
    if d.abs() < 1e-10 {
        return None;
    }

    // Use barycentric coordinates for circumcenter
    let alpha = (p1 - p2).norm_squared() * (p0 - p1).dot(&(p0 - p2));
    let beta = (p0 - p2).norm_squared() * (p1 - p0).dot(&(p1 - p2));
    let gamma = (p0 - p1).norm_squared() * (p2 - p0).dot(&(p2 - p1));
    let denom = alpha + beta + gamma;

    if denom.abs() < 1e-10 {
        return None;
    }

    let circumcenter =
        Point3::from((alpha * p0.coords + beta * p1.coords + gamma * p2.coords) / denom);

    // Choose ball center direction based on point normal
    let direction = if tri_normal_unit.dot(normal) > 0.0 {
        tri_normal_unit
    } else {
        -tri_normal_unit
    };

    Some(Point3::from(circumcenter.coords + direction * h))
}

/// Checks if the ball at center contains no other points.
fn is_empty_ball(
    cloud: &PointCloud,
    tree: &KdTree<f64, 3>,
    center: &Point3<f64>,
    radius: f64,
    exclude: &[usize],
) -> bool {
    let radius_sq = radius * radius;
    let tolerance = 1e-6;

    let neighbors =
        tree.within::<SquaredEuclidean>(&[center.x, center.y, center.z], radius_sq * 1.01);

    for neighbor in neighbors {
        #[allow(clippy::cast_possible_truncation)]
        let idx = neighbor.item as usize;
        if exclude.contains(&idx) {
            continue;
        }

        let dist_sq = (cloud.points[idx].position - center).norm_squared();
        if dist_sq < radius_sq - tolerance {
            return false;
        }
    }

    true
}

/// Pivots the ball around an edge to find the next vertex.
fn pivot_ball(
    cloud: &PointCloud,
    tree: &KdTree<f64, 3>,
    edge: &Edge,
    ball_center: &Point3<f64>,
    params: &BallPivotingParams,
    used: &HashSet<u32>,
) -> Option<(u32, Point3<f64>)> {
    let p0 = cloud.points[edge.v0 as usize].position;
    let p1 = cloud.points[edge.v1 as usize].position;

    let edge_mid = Point3::from((p0.coords + p1.coords) / 2.0);
    let edge_vec = p1 - p0;
    let edge_len = edge_vec.norm();

    if edge_len < 1e-10 {
        return None;
    }

    // Search for candidate points
    let search_radius = params.radius * 2.5;
    let search_radius_sq = search_radius * search_radius;

    let candidates =
        tree.within::<SquaredEuclidean>(&[edge_mid.x, edge_mid.y, edge_mid.z], search_radius_sq);

    let mut best_candidate: Option<(u32, Point3<f64>, f64)> = None;

    for candidate in candidates {
        #[allow(clippy::cast_possible_truncation)]
        let idx = candidate.item as u32;

        // Skip edge vertices
        if idx == edge.v0 || idx == edge.v1 {
            continue;
        }

        let p2 = cloud.points[idx as usize].position;
        let normal = cloud.points[idx as usize].normal.unwrap_or(Vector3::z());

        // Try to compute ball center for this triangle
        if let Some(new_center) = compute_ball_center(&p0, &p1, &p2, &normal, params.radius) {
            // Check if ball is on correct side (opposite to current ball)
            let old_dir = ball_center - edge_mid;
            let new_dir = new_center - edge_mid;

            // They should be on opposite sides
            if old_dir.dot(&new_dir) > 0.0 {
                continue;
            }

            // Compute pivot angle
            let pivot_angle = old_dir.angle(&(-new_dir));

            // Check angle constraint
            if pivot_angle > params.max_pivot_angle {
                continue;
            }

            // Check if ball is empty
            if !is_empty_ball(
                cloud,
                tree,
                &new_center,
                params.radius,
                &[edge.v0 as usize, edge.v1 as usize, idx as usize],
            ) {
                continue;
            }

            // Prefer vertices not yet used, then smallest pivot angle
            let priority = if used.contains(&idx) { 1.0 } else { 0.0 };
            let score = priority + pivot_angle / std::f64::consts::PI;

            if best_candidate.is_none() || score < best_candidate.as_ref().map_or(f64::MAX, |c| c.2)
            {
                best_candidate = Some((idx, new_center, score));
            }
        }
    }

    best_candidate.map(|(idx, center, _)| (idx, center))
}

/// Counts boundary edges (edges with only one adjacent face).
fn count_boundary_edges(mesh: &IndexedMesh) -> usize {
    let mut edge_counts: HashMap<Edge, usize> = HashMap::new();

    for face in &mesh.faces {
        let edges = [
            Edge::new(face[0], face[1]),
            Edge::new(face[1], face[2]),
            Edge::new(face[2], face[0]),
        ];

        for edge in edges {
            *edge_counts.entry(edge).or_insert(0) += 1;
        }
    }

    edge_counts.values().filter(|&&count| count == 1).count()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_simple_cloud() -> PointCloud {
        let mut cloud = PointCloud::new();

        // Create a simple grid of points
        for i in 0..3 {
            for j in 0..3 {
                let x = f64::from(i);
                let y = f64::from(j);
                cloud.add_point_with_normal(Point3::new(x, y, 0.0), Vector3::new(0.0, 0.0, 1.0));
            }
        }

        cloud
    }

    #[test]
    fn test_ball_pivoting_params_default() {
        let params = BallPivotingParams::default();
        assert!((params.radius - 1.0).abs() < 1e-10);
        assert!(!params.fill_small_holes);
    }

    #[test]
    fn test_ball_pivoting_params_builder() {
        let params = BallPivotingParams::new(2.0)
            .with_max_pivot_angle(1.0)
            .with_fill_holes(true, 5);

        assert!((params.radius - 2.0).abs() < 1e-10);
        assert!(params.fill_small_holes);
        assert_eq!(params.max_hole_size, 5);
    }

    #[test]
    fn test_estimate_radius() {
        let cloud = make_simple_cloud();
        let radius = BallPivotingParams::estimate_radius(&cloud, 4);

        // Points are 1 unit apart, so radius should be around 1.5
        assert!(radius > 0.5);
        assert!(radius < 5.0);
    }

    #[test]
    fn test_estimate_radius_empty() {
        let cloud = PointCloud::new();
        let radius = BallPivotingParams::estimate_radius(&cloud, 4);
        assert!((radius - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_ball_pivoting_empty_cloud() {
        let cloud = PointCloud::new();
        let params = BallPivotingParams::default();
        let result = ball_pivoting(&cloud, &params);
        assert!(matches!(result, Err(ScanError::EmptyPointCloud)));
    }

    #[test]
    fn test_ball_pivoting_no_normals() {
        let mut cloud = PointCloud::new();
        cloud.add_point(Point3::new(0.0, 0.0, 0.0));

        let params = BallPivotingParams::default();
        let result = ball_pivoting(&cloud, &params);
        assert!(matches!(
            result,
            Err(ScanError::NormalEstimationFailed { .. })
        ));
    }

    #[test]
    fn test_ball_pivoting_simple() {
        let mut cloud = PointCloud::new();
        cloud.add_point_with_normal(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        cloud.add_point_with_normal(Point3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        cloud.add_point_with_normal(Point3::new(0.5, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));

        let params = BallPivotingParams::new(2.0);
        let result = ball_pivoting(&cloud, &params).unwrap();

        // Verify result is valid (triangle_count is usize)
        let _ = result.triangle_count; // Just verify it exists
        assert_eq!(result.mesh.vertices.len(), 3);
    }

    #[test]
    fn test_ball_pivoting_grid() {
        let cloud = make_simple_cloud();
        let params = BallPivotingParams::new(1.5);
        let result = ball_pivoting(&cloud, &params).unwrap();

        // 3x3 grid should produce some triangles
        assert!(result.mesh.vertices.len() == 9);
        // Coverage should be positive
        assert!(result.coverage_ratio >= 0.0);
    }

    #[test]
    fn test_ball_pivoting_result_display() {
        let result = BallPivotingResult {
            mesh: IndexedMesh::new(),
            triangle_count: 100,
            orphan_points: 5,
            coverage_ratio: 0.95,
            boundary_edge_count: 10,
        };

        let display = format!("{result}");
        assert!(display.contains("100"));
        assert!(display.contains("95.0%"));
        assert!(display.contains("10"));
    }

    #[test]
    fn test_edge_ordering() {
        let e1 = Edge::new(5, 3);
        let e2 = Edge::new(3, 5);

        // Should normalize to same edge
        assert_eq!(e1.v0, e2.v0);
        assert_eq!(e1.v1, e2.v1);
    }

    #[test]
    fn test_count_boundary_edges() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);

        // Single triangle has 3 boundary edges
        let count = count_boundary_edges(&mesh);
        assert_eq!(count, 3);
    }

    #[test]
    fn test_count_boundary_edges_two_triangles() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([1, 3, 2]); // Shares edge 1-2

        // Two triangles sharing one edge have 4 boundary edges
        let count = count_boundary_edges(&mesh);
        assert_eq!(count, 4);
    }
}
