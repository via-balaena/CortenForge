//! Cross-section extraction from meshes.
//!
//! Computes plane-mesh intersections and extracts contours.

// Mesh processing uses u32 indices; truncation would only occur for meshes with >4B vertices
// which exceeds practical limits.
#![allow(clippy::cast_precision_loss)]

use mesh_types::{IndexedMesh, MeshTopology, Point3, Vector3};

/// Result of cross-section extraction.
///
/// Contains the intersection contour(s) and derived measurements.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_measure::cross_section;
/// use nalgebra::{Point3, Vector3};
///
/// let cube = unit_cube();
/// // Cross-section at z=0.5 (middle of cube)
/// let section = cross_section(&cube, Point3::new(0.0, 0.0, 0.5), Vector3::z());
///
/// // Should be a 1x1 square
/// assert!((section.area - 1.0).abs() < 0.1);
/// ```
#[derive(Debug, Clone)]
pub struct CrossSection {
    /// Points on the cross-section boundary (may contain multiple contours).
    pub points: Vec<Point3<f64>>,
    /// Perimeter length (total length of all contours).
    pub perimeter: f64,
    /// Area enclosed by the cross-section.
    pub area: f64,
    /// Centroid of the cross-section.
    pub centroid: Point3<f64>,
    /// Bounding box of the cross-section (min, max).
    pub bounds: (Point3<f64>, Point3<f64>),
    /// Plane origin point.
    pub plane_origin: Point3<f64>,
    /// Plane normal.
    pub plane_normal: Vector3<f64>,
    /// Number of separate contours found.
    pub contour_count: usize,
}

impl Default for CrossSection {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            perimeter: 0.0,
            area: 0.0,
            centroid: Point3::origin(),
            bounds: (Point3::origin(), Point3::origin()),
            plane_origin: Point3::origin(),
            plane_normal: Vector3::z(),
            contour_count: 0,
        }
    }
}

impl CrossSection {
    /// Check if the cross-section is empty (no intersection with mesh).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Check if the cross-section forms a closed contour.
    ///
    /// Returns true if there's at least one contour.
    #[must_use]
    pub const fn is_closed(&self) -> bool {
        self.contour_count > 0
    }
}

/// Extract a cross-section of the mesh at a given plane.
///
/// Computes the intersection of the mesh with a plane defined by
/// a point and normal vector.
///
/// # Arguments
///
/// * `mesh` - The mesh to slice
/// * `plane_point` - A point on the cutting plane
/// * `plane_normal` - The normal vector of the cutting plane
///
/// # Returns
///
/// A [`CrossSection`] containing the intersection contour(s) and measurements.
///
/// # Example
///
/// ```
/// use mesh_types::{IndexedMesh, Vertex};
/// use mesh_measure::cross_section;
/// use nalgebra::{Point3, Vector3};
///
/// // Create a simple tetrahedron
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(5.0, 5.0, 10.0));
/// mesh.faces.push([0, 1, 3]);
/// mesh.faces.push([1, 2, 3]);
/// mesh.faces.push([2, 0, 3]);
/// mesh.faces.push([0, 2, 1]);
///
/// let section = cross_section(&mesh, Point3::new(0.0, 0.0, 5.0), Vector3::z());
/// assert!(section.area > 0.0);
/// ```
#[must_use]
pub fn cross_section(
    mesh: &IndexedMesh,
    plane_point: Point3<f64>,
    plane_normal: Vector3<f64>,
) -> CrossSection {
    let normal = plane_normal.normalize();
    let mut segments: Vec<(Point3<f64>, Point3<f64>)> = Vec::new();

    // Find all edge intersections with the plane
    for triangle in mesh.triangles() {
        let v0 = triangle.v0;
        let v1 = triangle.v1;
        let v2 = triangle.v2;

        let mut intersections = Vec::new();

        // Check each edge
        for (a, b) in [(v0, v1), (v1, v2), (v2, v0)] {
            if let Some(p) = plane_edge_intersection(plane_point, normal, a, b) {
                intersections.push(p);
            }
        }

        // If we have exactly 2 intersections, we have a segment
        if intersections.len() == 2 {
            segments.push((intersections[0], intersections[1]));
        }
    }

    if segments.is_empty() {
        return CrossSection {
            plane_origin: plane_point,
            plane_normal: normal,
            ..CrossSection::default()
        };
    }

    // Chain segments into contours
    let contours = chain_segments(&segments);
    let contour_count = contours.len();

    // Flatten all points for calculations
    let all_points: Vec<Point3<f64>> = contours.into_iter().flatten().collect();

    // Calculate perimeter
    let mut perimeter = 0.0;
    for segment in &segments {
        perimeter += (segment.1 - segment.0).norm();
    }

    // Calculate area using the shoelace formula (projected to 2D on plane)
    let area = calculate_cross_section_area(&all_points, normal);

    // Calculate centroid
    let centroid = if all_points.is_empty() {
        plane_point
    } else {
        let sum: Vector3<f64> = all_points.iter().map(|p| p.coords).sum();
        Point3::from(sum / all_points.len() as f64)
    };

    // Calculate bounds
    let (min, max) = compute_points_bounds(&all_points);

    CrossSection {
        points: all_points,
        perimeter,
        area,
        centroid,
        bounds: (min, max),
        plane_origin: plane_point,
        plane_normal: normal,
        contour_count,
    }
}

/// Extract multiple cross-sections at regular intervals.
///
/// # Arguments
///
/// * `mesh` - The mesh to slice
/// * `start` - Starting point for the first slice
/// * `normal` - Normal direction (slices advance in this direction)
/// * `count` - Number of slices to extract
/// * `spacing` - Distance between slices
///
/// # Returns
///
/// A vector of [`CrossSection`] results, one per slice.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_measure::cross_sections;
/// use nalgebra::{Point3, Vector3};
///
/// let cube = unit_cube();
/// let sections = cross_sections(
///     &cube,
///     Point3::new(0.0, 0.0, 0.0),
///     Vector3::z(),
///     5,
///     0.25
/// );
///
/// assert_eq!(sections.len(), 5);
/// ```
#[must_use]
pub fn cross_sections(
    mesh: &IndexedMesh,
    start: Point3<f64>,
    normal: Vector3<f64>,
    count: usize,
    spacing: f64,
) -> Vec<CrossSection> {
    let normal = normal.normalize();
    let mut sections = Vec::with_capacity(count);

    for i in 0..count {
        let offset = i as f64 * spacing;
        let plane_point = Point3::from(start.coords + normal * offset);
        sections.push(cross_section(mesh, plane_point, normal));
    }

    sections
}

/// Measure circumference at a given height (Z coordinate).
///
/// This is a convenience function that extracts a horizontal cross-section
/// and returns its perimeter.
///
/// # Arguments
///
/// * `mesh` - The mesh to measure
/// * `z` - The Z height at which to measure
///
/// # Returns
///
/// The perimeter of the cross-section at the given height.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_measure::circumference_at_height;
///
/// let cube = unit_cube();
/// let circ = circumference_at_height(&cube, 0.5);
/// // Unit cube has perimeter 4.0 at any horizontal slice
/// assert!((circ - 4.0).abs() < 0.1);
/// ```
#[must_use]
pub fn circumference_at_height(mesh: &IndexedMesh, z: f64) -> f64 {
    let section = cross_section(mesh, Point3::new(0.0, 0.0, z), Vector3::z());
    section.perimeter
}

/// Measure area at a given height (Z coordinate).
///
/// This is a convenience function that extracts a horizontal cross-section
/// and returns its area.
///
/// # Arguments
///
/// * `mesh` - The mesh to measure
/// * `z` - The Z height at which to measure
///
/// # Returns
///
/// The area of the cross-section at the given height.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_measure::area_at_height;
///
/// let cube = unit_cube();
/// let area = area_at_height(&cube, 0.5);
/// // Unit cube has area 1.0 at any horizontal slice
/// assert!((area - 1.0).abs() < 0.1);
/// ```
#[must_use]
pub fn area_at_height(mesh: &IndexedMesh, z: f64) -> f64 {
    let section = cross_section(mesh, Point3::new(0.0, 0.0, z), Vector3::z());
    section.area
}

// ============================================================================
// Internal helper functions
// ============================================================================

fn plane_edge_intersection(
    plane_point: Point3<f64>,
    plane_normal: Vector3<f64>,
    a: Point3<f64>,
    b: Point3<f64>,
) -> Option<Point3<f64>> {
    let d_a = (a - plane_point).dot(&plane_normal);
    let d_b = (b - plane_point).dot(&plane_normal);

    // Check if edge crosses the plane
    if d_a * d_b > 0.0 {
        return None; // Same side of plane
    }

    if (d_a - d_b).abs() < 1e-10 {
        return None; // Edge parallel to plane
    }

    let t = d_a / (d_a - d_b);
    let direction = b - a;
    Some(Point3::from(a.coords + direction * t))
}

fn chain_segments(segments: &[(Point3<f64>, Point3<f64>)]) -> Vec<Vec<Point3<f64>>> {
    if segments.is_empty() {
        return Vec::new();
    }

    let mut remaining: Vec<_> = segments.to_vec();
    let mut contours = Vec::new();

    while !remaining.is_empty() {
        let mut contour = Vec::new();
        let first = remaining.remove(0);
        contour.push(first.0);
        contour.push(first.1);

        let mut changed = true;
        while changed {
            changed = false;

            // Copy endpoints to avoid borrow issues
            let start = contour[0];
            let end = contour[contour.len() - 1];
            let eps = 1e-6;

            for i in (0..remaining.len()).rev() {
                let seg = &remaining[i];

                if (seg.0 - end).norm() < eps {
                    contour.push(seg.1);
                    remaining.remove(i);
                    changed = true;
                } else if (seg.1 - end).norm() < eps {
                    contour.push(seg.0);
                    remaining.remove(i);
                    changed = true;
                } else if (seg.0 - start).norm() < eps {
                    contour.insert(0, seg.1);
                    remaining.remove(i);
                    changed = true;
                } else if (seg.1 - start).norm() < eps {
                    contour.insert(0, seg.0);
                    remaining.remove(i);
                    changed = true;
                }
            }
        }

        contours.push(contour);
    }

    contours
}

fn calculate_cross_section_area(points: &[Point3<f64>], normal: Vector3<f64>) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }

    // Project points onto 2D plane
    // Create orthonormal basis on the plane
    let u = if normal.x.abs() < 0.9 {
        Vector3::x().cross(&normal).normalize()
    } else {
        Vector3::y().cross(&normal).normalize()
    };
    let v = normal.cross(&u);

    // Project to 2D
    let points_2d: Vec<(f64, f64)> = points
        .iter()
        .map(|p| (p.coords.dot(&u), p.coords.dot(&v)))
        .collect();

    // Shoelace formula
    let mut area = 0.0;
    let n = points_2d.len();
    for i in 0..n {
        let j = (i + 1) % n;
        area += points_2d[i].0 * points_2d[j].1;
        area -= points_2d[j].0 * points_2d[i].1;
    }

    (area / 2.0).abs()
}

fn compute_points_bounds(points: &[Point3<f64>]) -> (Point3<f64>, Point3<f64>) {
    if points.is_empty() {
        return (Point3::origin(), Point3::origin());
    }

    let mut min = points[0];
    let mut max = points[0];

    for p in points {
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        min.z = min.z.min(p.z);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
        max.z = max.z.max(p.z);
    }

    (min, max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::{unit_cube, Vertex};

    fn create_test_cube(size: f64) -> IndexedMesh {
        let mut cube = unit_cube();
        cube.scale(size);
        cube
    }

    #[test]
    fn test_cross_section_cube_middle() {
        let cube = unit_cube();
        let section = cross_section(&cube, Point3::new(0.0, 0.0, 0.5), Vector3::z());

        // Should be a 1x1 square
        assert!((section.area - 1.0).abs() < 0.1);
        assert!((section.perimeter - 4.0).abs() < 0.1);
    }

    #[test]
    fn test_cross_section_cube_near_top() {
        let cube = create_test_cube(10.0);
        let section = cross_section(&cube, Point3::new(0.0, 0.0, 9.0), Vector3::z());

        // Should still be 10x10 = 100 sq units
        assert!((section.area - 100.0).abs() < 1.0);
    }

    #[test]
    fn test_cross_section_outside_mesh() {
        let cube = unit_cube();
        let section = cross_section(&cube, Point3::new(0.0, 0.0, 10.0), Vector3::z());

        assert!(section.is_empty());
        assert_eq!(section.contour_count, 0);
    }

    #[test]
    fn test_multiple_cross_sections() {
        let cube = unit_cube();
        let sections = cross_sections(&cube, Point3::new(0.0, 0.0, 0.1), Vector3::z(), 5, 0.2);

        assert_eq!(sections.len(), 5);
        for section in &sections {
            if !section.is_empty() {
                assert!((section.area - 1.0).abs() < 0.2);
            }
        }
    }

    #[test]
    fn test_circumference_at_height() {
        let cube = unit_cube();
        let circ = circumference_at_height(&cube, 0.5);

        assert!((circ - 4.0).abs() < 0.1);
    }

    #[test]
    fn test_area_at_height() {
        let cube = unit_cube();
        let area = area_at_height(&cube, 0.5);

        assert!((area - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_cross_section_tetrahedron() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(10.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(5.0, 10.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(5.0, 5.0, 10.0));
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh.faces.push([2, 0, 3]);
        mesh.faces.push([0, 2, 1]);

        let section = cross_section(&mesh, Point3::new(0.0, 0.0, 5.0), Vector3::z());

        // At z=5, the tetrahedron has a smaller triangular cross-section
        assert!(section.area > 0.0);
        assert!(section.contour_count >= 1);
    }

    #[test]
    fn test_cross_section_non_z_plane() {
        let cube = unit_cube();
        // Slice diagonally
        let section = cross_section(
            &cube,
            Point3::new(0.5, 0.5, 0.5),
            Vector3::new(1.0, 1.0, 1.0).normalize(),
        );

        // Should have some area
        assert!(section.area > 0.0);
    }

    #[test]
    fn test_cross_section_y_plane() {
        let cube = unit_cube();
        let section = cross_section(&cube, Point3::new(0.0, 0.5, 0.0), Vector3::y());

        // Should be a 1x1 square
        assert!((section.area - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_cross_section_centroid() {
        let cube = unit_cube();
        let section = cross_section(&cube, Point3::new(0.0, 0.0, 0.5), Vector3::z());

        // Centroid should be approximately at center of unit cube's z=0.5 slice
        if !section.is_empty() {
            assert!((section.centroid.x - 0.5).abs() < 0.2);
            assert!((section.centroid.y - 0.5).abs() < 0.2);
        }
    }
}
