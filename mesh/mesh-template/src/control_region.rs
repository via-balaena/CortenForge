//! Control region definitions for template fitting.
//!
//! Control regions define areas of the mesh that can be manipulated during fitting.
//! They can represent single landmarks, vertex groups, or geometric regions.

use crate::MeasurementType;
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};
use std::collections::HashSet;

/// Definition of a region's geometric extent.
///
/// Different region types are suitable for different use cases:
///
/// | Type | Use Case |
/// |------|----------|
/// | `Point` | Anatomical landmarks, single control points |
/// | `Vertices` | Specific vertex selections |
/// | `Faces` | Face-based selections |
/// | `Bounds` | Rectangular regions |
/// | `Sphere` | Localized circular areas |
/// | `Cylinder` | Limbs, tubes, elongated regions |
/// | `MeasurementPlane` | Cross-sections for measurements |
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum RegionDefinition {
    /// A single point landmark.
    Point(Point3<f64>),

    /// A set of vertex indices.
    Vertices(Vec<u32>),

    /// A set of face indices.
    Faces(Vec<u32>),

    /// An axis-aligned bounding box region.
    Bounds {
        /// Minimum corner of the box.
        min: Point3<f64>,
        /// Maximum corner of the box.
        max: Point3<f64>,
    },

    /// A spherical region.
    Sphere {
        /// Center of the sphere.
        center: Point3<f64>,
        /// Radius of the sphere.
        radius: f64,
    },

    /// A cylindrical region.
    Cylinder {
        /// Start point of the cylinder axis.
        axis_start: Point3<f64>,
        /// End point of the cylinder axis.
        axis_end: Point3<f64>,
        /// Radius of the cylinder.
        radius: f64,
    },

    /// A measurement plane for extracting dimensions.
    MeasurementPlane {
        /// Type of measurement to extract.
        measurement_type: MeasurementType,
        /// Origin point of the measurement plane.
        origin: Point3<f64>,
        /// Normal vector of the measurement plane.
        normal: Vector3<f64>,
    },
}

/// A named control region with properties.
///
/// Control regions define manipulable areas on the template mesh.
/// Each region has:
/// - A unique name for reference
/// - A geometric definition
/// - A weight for importance during fitting
/// - A preservation flag to prevent deformation
///
/// # Examples
///
/// ```
/// use mesh_template::{ControlRegion, MeasurementType};
/// use nalgebra::{Point3, Vector3};
///
/// // Point landmark
/// let tip = ControlRegion::point("nose_tip", Point3::new(0.0, 0.0, 10.0));
///
/// // Vertex selection
/// let base = ControlRegion::vertices("base_vertices", vec![0, 1, 2, 3]);
///
/// // Spherical region with increased weight
/// let eye = ControlRegion::sphere("eye_region", Point3::new(3.0, 0.0, 5.0), 2.0)
///     .with_weight(2.0);
///
/// // Measurement region
/// let waist = ControlRegion::measurement(
///     "waist_circumference",
///     MeasurementType::Circumference,
///     Point3::new(0.0, 0.0, 50.0),
///     Vector3::z(),
/// );
/// ```
#[derive(Debug, Clone)]
pub struct ControlRegion {
    /// Name of the control region.
    pub name: String,

    /// Geometric definition of the region.
    pub definition: RegionDefinition,

    /// Weight for this region during fitting (default: 1.0).
    ///
    /// Higher weights give this region more influence in the fit.
    pub weight: f64,

    /// Whether this region should be preserved (not deformed).
    ///
    /// Preserved regions act as anchors during morphing.
    pub preserve: bool,
}

impl ControlRegion {
    /// Creates a point landmark region.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::ControlRegion;
    /// use nalgebra::Point3;
    ///
    /// let landmark = ControlRegion::point("apex", Point3::new(0.0, 0.0, 10.0));
    /// assert_eq!(landmark.name, "apex");
    /// ```
    #[must_use]
    pub fn point(name: impl Into<String>, position: Point3<f64>) -> Self {
        Self {
            name: name.into(),
            definition: RegionDefinition::Point(position),
            weight: 1.0,
            preserve: false,
        }
    }

    /// Creates a vertex-based region.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::ControlRegion;
    ///
    /// let region = ControlRegion::vertices("boundary", vec![0, 1, 2, 3]);
    /// ```
    #[must_use]
    pub fn vertices(name: impl Into<String>, indices: Vec<u32>) -> Self {
        Self {
            name: name.into(),
            definition: RegionDefinition::Vertices(indices),
            weight: 1.0,
            preserve: false,
        }
    }

    /// Creates a face-based region.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::ControlRegion;
    ///
    /// let region = ControlRegion::faces("top_surface", vec![0, 1, 2]);
    /// ```
    #[must_use]
    pub fn faces(name: impl Into<String>, face_indices: Vec<u32>) -> Self {
        Self {
            name: name.into(),
            definition: RegionDefinition::Faces(face_indices),
            weight: 1.0,
            preserve: false,
        }
    }

    /// Creates an axis-aligned bounding box region.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::ControlRegion;
    /// use nalgebra::Point3;
    ///
    /// let region = ControlRegion::bounds(
    ///     "front_half",
    ///     Point3::new(-10.0, -10.0, 0.0),
    ///     Point3::new(10.0, 0.0, 20.0),
    /// );
    /// ```
    #[must_use]
    pub fn bounds(name: impl Into<String>, min: Point3<f64>, max: Point3<f64>) -> Self {
        Self {
            name: name.into(),
            definition: RegionDefinition::Bounds { min, max },
            weight: 1.0,
            preserve: false,
        }
    }

    /// Creates a spherical region.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::ControlRegion;
    /// use nalgebra::Point3;
    ///
    /// let region = ControlRegion::sphere("head", Point3::new(0.0, 0.0, 100.0), 15.0);
    /// ```
    #[must_use]
    pub fn sphere(name: impl Into<String>, center: Point3<f64>, radius: f64) -> Self {
        Self {
            name: name.into(),
            definition: RegionDefinition::Sphere { center, radius },
            weight: 1.0,
            preserve: false,
        }
    }

    /// Creates a cylindrical region.
    ///
    /// The cylinder extends from `axis_start` to `axis_end` with the given radius.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::ControlRegion;
    /// use nalgebra::Point3;
    ///
    /// // Cylinder along the Z axis
    /// let region = ControlRegion::cylinder(
    ///     "torso",
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(0.0, 0.0, 100.0),
    ///     20.0,
    /// );
    /// ```
    #[must_use]
    pub fn cylinder(
        name: impl Into<String>,
        axis_start: Point3<f64>,
        axis_end: Point3<f64>,
        radius: f64,
    ) -> Self {
        Self {
            name: name.into(),
            definition: RegionDefinition::Cylinder {
                axis_start,
                axis_end,
                radius,
            },
            weight: 1.0,
            preserve: false,
        }
    }

    /// Creates a measurement plane region.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::{ControlRegion, MeasurementType};
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let region = ControlRegion::measurement(
    ///     "chest_circumference",
    ///     MeasurementType::Circumference,
    ///     Point3::new(0.0, 0.0, 120.0),
    ///     Vector3::z(),
    /// );
    /// ```
    #[must_use]
    pub fn measurement(
        name: impl Into<String>,
        measurement_type: MeasurementType,
        origin: Point3<f64>,
        normal: Vector3<f64>,
    ) -> Self {
        Self {
            name: name.into(),
            definition: RegionDefinition::MeasurementPlane {
                measurement_type,
                origin,
                normal: normal.normalize(),
            },
            weight: 1.0,
            preserve: false,
        }
    }

    /// Sets the weight for this region.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::ControlRegion;
    /// use nalgebra::Point3;
    ///
    /// let region = ControlRegion::point("important", Point3::origin())
    ///     .with_weight(2.0);
    /// assert!((region.weight - 2.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub const fn with_weight(mut self, weight: f64) -> Self {
        self.weight = weight;
        self
    }

    /// Marks this region as preserved (will not be deformed).
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::ControlRegion;
    /// use nalgebra::Point3;
    ///
    /// let region = ControlRegion::point("anchor", Point3::origin())
    ///     .preserved();
    /// assert!(region.preserve);
    /// ```
    #[must_use]
    pub const fn preserved(mut self) -> Self {
        self.preserve = true;
        self
    }

    /// Gets the vertex indices belonging to this region.
    ///
    /// This method queries the mesh to find all vertices that fall within
    /// the region's geometric definition.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::ControlRegion;
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use nalgebra::Point3;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(2.0, 0.0, 0.0));
    ///
    /// let region = ControlRegion::sphere("near_origin", Point3::origin(), 1.5);
    /// let indices = region.get_vertex_indices(&mesh);
    ///
    /// assert!(indices.contains(&0));
    /// assert!(indices.contains(&1));
    /// assert!(!indices.contains(&2)); // Too far from origin
    /// ```
    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn get_vertex_indices(&self, mesh: &IndexedMesh) -> HashSet<u32> {
        match &self.definition {
            RegionDefinition::Point(pos) => {
                // Find the closest vertex to this point
                let mut closest_idx = 0u32;
                let mut closest_dist = f64::MAX;

                for (i, v) in mesh.vertices.iter().enumerate() {
                    let dist = (v.position - pos).norm();
                    if dist < closest_dist {
                        closest_dist = dist;
                        #[allow(clippy::cast_possible_truncation)]
                        {
                            closest_idx = i as u32;
                        }
                    }
                }

                let mut set = HashSet::new();
                if !mesh.vertices.is_empty() {
                    set.insert(closest_idx);
                }
                set
            }

            RegionDefinition::Vertices(indices) => indices.iter().copied().collect(),

            RegionDefinition::Faces(face_indices) => {
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

            RegionDefinition::Bounds { min, max } => {
                let mut vertices = HashSet::new();
                for (i, v) in mesh.vertices.iter().enumerate() {
                    let p = &v.position;
                    if p.x >= min.x
                        && p.x <= max.x
                        && p.y >= min.y
                        && p.y <= max.y
                        && p.z >= min.z
                        && p.z <= max.z
                    {
                        #[allow(clippy::cast_possible_truncation)]
                        vertices.insert(i as u32);
                    }
                }
                vertices
            }

            RegionDefinition::Sphere { center, radius } => {
                let radius_sq = radius * radius;
                let mut vertices = HashSet::new();
                for (i, v) in mesh.vertices.iter().enumerate() {
                    let dist_sq = (v.position - center).norm_squared();
                    if dist_sq <= radius_sq {
                        #[allow(clippy::cast_possible_truncation)]
                        vertices.insert(i as u32);
                    }
                }
                vertices
            }

            RegionDefinition::Cylinder {
                axis_start,
                axis_end,
                radius,
            } => {
                let axis = axis_end - axis_start;
                let axis_len_sq = axis.norm_squared();

                if axis_len_sq < 1e-10 {
                    // Degenerate cylinder, treat as point
                    return self.get_vertex_indices(mesh);
                }

                let radius_sq = radius * radius;
                let mut vertices = HashSet::new();

                for (i, v) in mesh.vertices.iter().enumerate() {
                    let to_point = v.position - axis_start;
                    let t = to_point.dot(&axis) / axis_len_sq;

                    // Check if projection is within cylinder extent
                    if (0.0..=1.0).contains(&t) {
                        // Compute distance from axis
                        let projection = axis_start + axis * t;
                        let dist_sq = (v.position - projection).norm_squared();

                        if dist_sq <= radius_sq {
                            #[allow(clippy::cast_possible_truncation)]
                            vertices.insert(i as u32);
                        }
                    }
                }
                vertices
            }

            RegionDefinition::MeasurementPlane { origin, normal, .. } => {
                // Find vertices near the measurement plane
                let tolerance = 1.0; // 1mm tolerance for plane intersection

                let mut vertices = HashSet::new();
                for (i, v) in mesh.vertices.iter().enumerate() {
                    let to_point = v.position - origin;
                    let dist = to_point.dot(normal).abs();

                    if dist <= tolerance {
                        #[allow(clippy::cast_possible_truncation)]
                        vertices.insert(i as u32);
                    }
                }
                vertices
            }
        }
    }

    /// Gets the centroid position of this region.
    ///
    /// For point regions, returns the point itself.
    /// For other regions, returns the center of the bounding geometry.
    #[must_use]
    pub fn centroid(&self) -> Point3<f64> {
        match &self.definition {
            RegionDefinition::Point(pos) => *pos,
            RegionDefinition::Vertices(_) | RegionDefinition::Faces(_) => {
                // Would need mesh to compute actual centroid
                Point3::origin()
            }
            RegionDefinition::Bounds { min, max } => Point3::from((min.coords + max.coords) / 2.0),
            RegionDefinition::Sphere { center, .. } => *center,
            RegionDefinition::Cylinder {
                axis_start,
                axis_end,
                ..
            } => Point3::from((axis_start.coords + axis_end.coords) / 2.0),
            RegionDefinition::MeasurementPlane { origin, .. } => *origin,
        }
    }

    /// Gets the centroid position of this region using actual mesh vertices.
    ///
    /// This computes the true centroid of the vertices within the region.
    #[must_use]
    pub fn centroid_in_mesh(&self, mesh: &IndexedMesh) -> Option<Point3<f64>> {
        let indices = self.get_vertex_indices(mesh);
        if indices.is_empty() {
            return None;
        }

        let mut sum = Vector3::zeros();
        for &idx in &indices {
            if let Some(v) = mesh.vertices.get(idx as usize) {
                sum += v.position.coords;
            }
        }

        #[allow(clippy::cast_precision_loss)]
        let centroid = sum / indices.len() as f64;
        Some(Point3::from(centroid))
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    fn make_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();

        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 1.0, 1.0));
        mesh.vertices.push(Vertex::from_coords(0.0, 1.0, 1.0));

        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);
        mesh
    }

    #[test]
    fn test_point_region() {
        let region = ControlRegion::point("test", Point3::new(1.0, 2.0, 3.0));
        assert_eq!(region.name, "test");
        assert_relative_eq!(region.weight, 1.0);
        assert!(!region.preserve);

        if let RegionDefinition::Point(p) = region.definition {
            assert_relative_eq!(p.x, 1.0);
            assert_relative_eq!(p.y, 2.0);
            assert_relative_eq!(p.z, 3.0);
        } else {
            panic!("Expected Point definition");
        }
    }

    #[test]
    fn test_vertices_region() {
        let region = ControlRegion::vertices("test", vec![1, 2, 3]);
        if let RegionDefinition::Vertices(v) = region.definition {
            assert_eq!(v, vec![1, 2, 3]);
        } else {
            panic!("Expected Vertices definition");
        }
    }

    #[test]
    fn test_faces_region() {
        let region = ControlRegion::faces("test", vec![0, 1]);
        if let RegionDefinition::Faces(f) = region.definition {
            assert_eq!(f, vec![0, 1]);
        } else {
            panic!("Expected Faces definition");
        }
    }

    #[test]
    fn test_bounds_region() {
        let region = ControlRegion::bounds(
            "test",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        );

        if let RegionDefinition::Bounds { min, max } = region.definition {
            assert_relative_eq!(min.x, 0.0);
            assert_relative_eq!(max.x, 1.0);
        } else {
            panic!("Expected Bounds definition");
        }
    }

    #[test]
    fn test_sphere_region() {
        let region = ControlRegion::sphere("test", Point3::new(1.0, 2.0, 3.0), 5.0);

        if let RegionDefinition::Sphere { center, radius } = region.definition {
            assert_relative_eq!(center.x, 1.0);
            assert_relative_eq!(radius, 5.0);
        } else {
            panic!("Expected Sphere definition");
        }
    }

    #[test]
    fn test_cylinder_region() {
        let region = ControlRegion::cylinder(
            "test",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 10.0),
            2.0,
        );

        if let RegionDefinition::Cylinder {
            axis_start,
            axis_end,
            radius,
        } = region.definition
        {
            assert_relative_eq!(axis_start.z, 0.0);
            assert_relative_eq!(axis_end.z, 10.0);
            assert_relative_eq!(radius, 2.0);
        } else {
            panic!("Expected Cylinder definition");
        }
    }

    #[test]
    fn test_measurement_region() {
        let region = ControlRegion::measurement(
            "test",
            MeasurementType::Circumference,
            Point3::new(0.0, 0.0, 5.0),
            Vector3::z(),
        );

        if let RegionDefinition::MeasurementPlane {
            measurement_type,
            origin,
            normal,
        } = region.definition
        {
            assert_eq!(measurement_type, MeasurementType::Circumference);
            assert_relative_eq!(origin.z, 5.0);
            assert_relative_eq!(normal.z, 1.0);
        } else {
            panic!("Expected MeasurementPlane definition");
        }
    }

    #[test]
    fn test_with_weight() {
        let region = ControlRegion::point("test", Point3::origin()).with_weight(2.5);
        assert_relative_eq!(region.weight, 2.5);
    }

    #[test]
    fn test_preserved() {
        let region = ControlRegion::point("test", Point3::origin()).preserved();
        assert!(region.preserve);
    }

    #[test]
    fn test_get_vertex_indices_point() {
        let mesh = make_cube();
        let region = ControlRegion::point("test", Point3::new(0.0, 0.0, 0.0));
        let indices = region.get_vertex_indices(&mesh);

        assert_eq!(indices.len(), 1);
        assert!(indices.contains(&0));
    }

    #[test]
    fn test_get_vertex_indices_vertices() {
        let mesh = make_cube();
        let region = ControlRegion::vertices("test", vec![0, 2, 4]);
        let indices = region.get_vertex_indices(&mesh);

        assert_eq!(indices.len(), 3);
        assert!(indices.contains(&0));
        assert!(indices.contains(&2));
        assert!(indices.contains(&4));
    }

    #[test]
    fn test_get_vertex_indices_faces() {
        let mesh = make_cube();
        let region = ControlRegion::faces("test", vec![0]); // First face [0, 1, 2]
        let indices = region.get_vertex_indices(&mesh);

        assert_eq!(indices.len(), 3);
        assert!(indices.contains(&0));
        assert!(indices.contains(&1));
        assert!(indices.contains(&2));
    }

    #[test]
    fn test_get_vertex_indices_bounds() {
        let mesh = make_cube();
        let region = ControlRegion::bounds(
            "test",
            Point3::new(-0.1, -0.1, -0.1),
            Point3::new(0.5, 0.5, 0.5),
        );
        let indices = region.get_vertex_indices(&mesh);

        // Should include vertex 0 at (0,0,0)
        assert!(indices.contains(&0));
        // Should not include vertex 6 at (1,1,1)
        assert!(!indices.contains(&6));
    }

    #[test]
    fn test_get_vertex_indices_sphere() {
        let mesh = make_cube();
        let region = ControlRegion::sphere("test", Point3::new(0.5, 0.5, 0.5), 1.0);
        let indices = region.get_vertex_indices(&mesh);

        // All cube corners are within distance ~0.866 from center
        assert_eq!(indices.len(), 8);
    }

    #[test]
    fn test_get_vertex_indices_cylinder() {
        let mesh = make_cube();
        let region = ControlRegion::cylinder(
            "test",
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(0.5, 0.5, 1.5),
            0.8,
        );
        let indices = region.get_vertex_indices(&mesh);

        // All vertices are within 0.707 of the z-axis through (0.5, 0.5)
        assert_eq!(indices.len(), 8);
    }

    #[test]
    fn test_get_vertex_indices_measurement_plane() {
        let mesh = make_cube();
        // Use a plane at z=5.0 which is far from any cube vertex (cube at z=0 and z=1)
        let region = ControlRegion::measurement(
            "test",
            MeasurementType::Circumference,
            Point3::new(0.0, 0.0, 5.0),
            Vector3::z(),
        );
        let indices = region.get_vertex_indices(&mesh);

        // No vertices are at z=5.0 within 1mm tolerance (z=0 and z=1)
        assert!(indices.is_empty());
    }

    #[test]
    fn test_centroid_point() {
        let region = ControlRegion::point("test", Point3::new(1.0, 2.0, 3.0));
        let centroid = region.centroid();
        assert_relative_eq!(centroid.x, 1.0);
        assert_relative_eq!(centroid.y, 2.0);
        assert_relative_eq!(centroid.z, 3.0);
    }

    #[test]
    fn test_centroid_bounds() {
        let region = ControlRegion::bounds(
            "test",
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 4.0, 6.0),
        );
        let centroid = region.centroid();
        assert_relative_eq!(centroid.x, 1.0);
        assert_relative_eq!(centroid.y, 2.0);
        assert_relative_eq!(centroid.z, 3.0);
    }

    #[test]
    fn test_centroid_in_mesh() {
        let mesh = make_cube();
        let region = ControlRegion::vertices("test", vec![0, 6]); // (0,0,0) and (1,1,1)
        let centroid = region.centroid_in_mesh(&mesh).unwrap();

        assert_relative_eq!(centroid.x, 0.5);
        assert_relative_eq!(centroid.y, 0.5);
        assert_relative_eq!(centroid.z, 0.5);
    }

    #[test]
    fn test_centroid_in_mesh_empty() {
        let mesh = make_cube();
        let region = ControlRegion::vertices("test", vec![]);
        assert!(region.centroid_in_mesh(&mesh).is_none());
    }

    #[test]
    fn test_measurement_plane_normalizes() {
        let region = ControlRegion::measurement(
            "test",
            MeasurementType::Width,
            Point3::origin(),
            Vector3::new(2.0, 0.0, 0.0), // Non-unit vector
        );

        if let RegionDefinition::MeasurementPlane { normal, .. } = region.definition {
            assert_relative_eq!(normal.norm(), 1.0, epsilon = 1e-10);
        } else {
            panic!("Expected MeasurementPlane");
        }
    }
}
