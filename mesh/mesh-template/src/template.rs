//! Template mesh with control regions.

use crate::{ControlRegion, FitParams, FitResult, RegionDefinition, TemplateResult};
use mesh_types::IndexedMesh;
use nalgebra::Point3;
use std::collections::HashMap;

/// A template mesh with named control regions for fitting.
///
/// `FitTemplate` combines a mesh with control regions that define
/// manipulable landmarks and measurement locations. The template
/// can be fit to scans, landmark targets, or measurement constraints.
///
/// # Examples
///
/// ## Creating a template
///
/// ```
/// use mesh_template::{FitTemplate, ControlRegion};
/// use mesh_types::{IndexedMesh, Vertex};
/// use nalgebra::Point3;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let template = FitTemplate::new(mesh)
///     .with_control_region(ControlRegion::point("tip", Point3::new(0.5, 1.0, 0.0)))
///     .with_control_region(ControlRegion::vertices("base", vec![0, 1]));
///
/// assert_eq!(template.region_names().len(), 2);
/// ```
///
/// ## Querying regions
///
/// ```
/// use mesh_template::{FitTemplate, ControlRegion};
/// use mesh_types::{IndexedMesh, Vertex};
/// use nalgebra::Point3;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 10.0));
/// mesh.faces.push([0, 0, 0]); // Degenerate but valid for testing
///
/// let template = FitTemplate::new(mesh)
///     .with_control_region(ControlRegion::point("apex", Point3::new(0.0, 0.0, 10.0)));
///
/// let pos = template.get_landmark_position("apex").unwrap();
/// assert!((pos.z - 10.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone)]
pub struct FitTemplate {
    /// The template mesh.
    pub mesh: IndexedMesh,

    /// Named control regions.
    control_regions: HashMap<String, ControlRegion>,

    /// Default fitting parameters.
    default_params: FitParams,
}

impl FitTemplate {
    /// Creates a new template from a mesh.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::FitTemplate;
    /// use mesh_types::{IndexedMesh, Vertex};
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    ///
    /// let template = FitTemplate::new(mesh);
    /// assert!(template.region_names().is_empty());
    /// ```
    #[must_use]
    pub fn new(mesh: IndexedMesh) -> Self {
        Self {
            mesh,
            control_regions: HashMap::new(),
            default_params: FitParams::default(),
        }
    }

    /// Adds a control region to the template.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::{FitTemplate, ControlRegion};
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use nalgebra::Point3;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    ///
    /// let template = FitTemplate::new(mesh)
    ///     .with_control_region(ControlRegion::point("center", Point3::origin()));
    ///
    /// assert!(template.get_region("center").is_some());
    /// ```
    #[must_use]
    pub fn with_control_region(mut self, region: ControlRegion) -> Self {
        self.control_regions.insert(region.name.clone(), region);
        self
    }

    /// Adds multiple control regions to the template.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::{FitTemplate, ControlRegion};
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use nalgebra::Point3;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    ///
    /// let regions = vec![
    ///     ControlRegion::point("a", Point3::new(0.0, 0.0, 0.0)),
    ///     ControlRegion::point("b", Point3::new(1.0, 0.0, 0.0)),
    /// ];
    ///
    /// let template = FitTemplate::new(mesh).with_control_regions(regions);
    /// assert_eq!(template.region_names().len(), 2);
    /// ```
    #[must_use]
    pub fn with_control_regions(
        mut self,
        regions: impl IntoIterator<Item = ControlRegion>,
    ) -> Self {
        for region in regions {
            self.control_regions.insert(region.name.clone(), region);
        }
        self
    }

    /// Sets the default fitting parameters.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::{FitTemplate, FitParams};
    /// use mesh_types::{IndexedMesh, Vertex};
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    ///
    /// let template = FitTemplate::new(mesh)
    ///     .with_default_params(FitParams::new().with_smoothness(0.5));
    /// ```
    #[must_use]
    pub fn with_default_params(mut self, params: FitParams) -> Self {
        self.default_params = params;
        self
    }

    /// Gets a control region by name.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::{FitTemplate, ControlRegion};
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use nalgebra::Point3;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    ///
    /// let template = FitTemplate::new(mesh)
    ///     .with_control_region(ControlRegion::point("test", Point3::origin()));
    ///
    /// assert!(template.get_region("test").is_some());
    /// assert!(template.get_region("missing").is_none());
    /// ```
    #[must_use]
    pub fn get_region(&self, name: &str) -> Option<&ControlRegion> {
        self.control_regions.get(name)
    }

    /// Gets the position of a point landmark.
    ///
    /// Returns `None` if the region doesn't exist or isn't a point.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::{FitTemplate, ControlRegion};
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use nalgebra::Point3;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    ///
    /// let template = FitTemplate::new(mesh)
    ///     .with_control_region(ControlRegion::point("tip", Point3::new(1.0, 2.0, 3.0)));
    ///
    /// let pos = template.get_landmark_position("tip").unwrap();
    /// assert!((pos.x - 1.0).abs() < 1e-10);
    /// ```
    #[must_use]
    pub fn get_landmark_position(&self, name: &str) -> Option<Point3<f64>> {
        let region = self.control_regions.get(name)?;
        match &region.definition {
            RegionDefinition::Point(pos) => Some(*pos),
            _ => region.centroid_in_mesh(&self.mesh),
        }
    }

    /// Returns the names of all control regions.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::{FitTemplate, ControlRegion};
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use nalgebra::Point3;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    ///
    /// let template = FitTemplate::new(mesh)
    ///     .with_control_region(ControlRegion::point("a", Point3::origin()))
    ///     .with_control_region(ControlRegion::point("b", Point3::origin()));
    ///
    /// let names = template.region_names();
    /// assert_eq!(names.len(), 2);
    /// ```
    #[must_use]
    pub fn region_names(&self) -> Vec<&str> {
        self.control_regions.keys().map(String::as_str).collect()
    }

    /// Returns the number of control regions.
    #[must_use]
    pub fn region_count(&self) -> usize {
        self.control_regions.len()
    }

    /// Fits the template using the provided parameters.
    ///
    /// This is the main fitting method that implements the three-stage pipeline:
    /// 1. Rigid alignment (if target scan provided)
    /// 2. Landmark deformation (if landmark targets provided)
    /// 3. Measurement adjustment (if measurement targets provided)
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - The template mesh is empty
    /// - No constraints are provided
    /// - A referenced region doesn't exist
    /// - The morphing or registration operation fails
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::{FitTemplate, FitParams, ControlRegion};
    /// use mesh_types::{IndexedMesh, Vertex};
    /// use nalgebra::Point3;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
    /// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
    /// mesh.faces.push([0, 1, 2]);
    ///
    /// let template = FitTemplate::new(mesh)
    ///     .with_control_region(ControlRegion::point("tip", Point3::new(0.5, 1.0, 0.0)));
    ///
    /// let params = FitParams::new()
    ///     .with_landmark_target("tip", Point3::new(0.5, 1.0, 0.0));
    ///
    /// let result = template.fit(&params).unwrap();
    /// assert!(result.fit_error < 0.1);
    /// ```
    pub fn fit(&self, params: &FitParams) -> TemplateResult<FitResult> {
        crate::fit::fit_template(self, params)
    }

    /// Convenience method to fit the template to a scan.
    ///
    /// Equivalent to `fit(&FitParams::new().with_target_scan(scan))`.
    ///
    /// # Errors
    ///
    /// Returns an error if the template or scan is empty, or if registration fails.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::FitTemplate;
    /// use mesh_types::{IndexedMesh, Vertex};
    ///
    /// let mut template_mesh = IndexedMesh::new();
    /// template_mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    /// template_mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
    /// template_mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
    /// template_mesh.faces.push([0, 1, 2]);
    ///
    /// let mut scan_mesh = template_mesh.clone();
    ///
    /// let template = FitTemplate::new(template_mesh);
    /// let result = template.fit_to_scan(&scan_mesh).unwrap();
    /// ```
    pub fn fit_to_scan(&self, scan: &IndexedMesh) -> TemplateResult<FitResult> {
        let params = FitParams::new().with_target_scan(scan.clone());
        self.fit(&params)
    }

    /// Convenience method to fit the template to measurement targets.
    ///
    /// # Errors
    ///
    /// Returns an error if the template is empty or if referenced regions don't exist.
    pub fn fit_to_measurements(
        &self,
        measurements: impl IntoIterator<Item = (String, crate::Measurement)>,
    ) -> TemplateResult<FitResult> {
        let params = FitParams::new().with_measurements(measurements);
        self.fit(&params)
    }

    /// Returns a reference to the internal control regions map.
    #[must_use]
    pub const fn control_regions(&self) -> &HashMap<String, ControlRegion> {
        &self.control_regions
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::Measurement;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    fn make_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    #[test]
    fn test_new() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh);

        assert!(template.control_regions.is_empty());
        assert_eq!(template.mesh.vertices.len(), 3);
    }

    #[test]
    fn test_with_control_region() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh)
            .with_control_region(ControlRegion::point("tip", Point3::new(0.5, 1.0, 0.0)));

        assert_eq!(template.control_regions.len(), 1);
        assert!(template.get_region("tip").is_some());
    }

    #[test]
    fn test_with_control_regions() {
        let mesh = make_triangle();
        let regions = vec![
            ControlRegion::point("a", Point3::origin()),
            ControlRegion::point("b", Point3::origin()),
            ControlRegion::point("c", Point3::origin()),
        ];

        let template = FitTemplate::new(mesh).with_control_regions(regions);
        assert_eq!(template.region_count(), 3);
    }

    #[test]
    fn test_with_default_params() {
        let mesh = make_triangle();
        let template =
            FitTemplate::new(mesh).with_default_params(FitParams::new().with_smoothness(0.5));

        assert_relative_eq!(template.default_params.smoothness, 0.5);
    }

    #[test]
    fn test_get_region() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh)
            .with_control_region(ControlRegion::point("test", Point3::origin()));

        assert!(template.get_region("test").is_some());
        assert!(template.get_region("missing").is_none());
    }

    #[test]
    fn test_get_landmark_position_point() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh)
            .with_control_region(ControlRegion::point("tip", Point3::new(1.0, 2.0, 3.0)));

        let pos = template.get_landmark_position("tip").unwrap();
        assert_relative_eq!(pos.x, 1.0);
        assert_relative_eq!(pos.y, 2.0);
        assert_relative_eq!(pos.z, 3.0);
    }

    #[test]
    fn test_get_landmark_position_vertices() {
        let mesh = make_triangle();
        let template =
            FitTemplate::new(mesh).with_control_region(ControlRegion::vertices("base", vec![0, 1]));

        // Centroid of vertices 0 (0,0,0) and 1 (1,0,0) is (0.5, 0, 0)
        let pos = template.get_landmark_position("base").unwrap();
        assert_relative_eq!(pos.x, 0.5);
        assert_relative_eq!(pos.y, 0.0);
    }

    #[test]
    fn test_get_landmark_position_missing() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh);

        assert!(template.get_landmark_position("missing").is_none());
    }

    #[test]
    fn test_region_names() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh)
            .with_control_region(ControlRegion::point("a", Point3::origin()))
            .with_control_region(ControlRegion::point("b", Point3::origin()));

        let names = template.region_names();
        assert_eq!(names.len(), 2);
        assert!(names.contains(&"a"));
        assert!(names.contains(&"b"));
    }

    #[test]
    fn test_region_count() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh);
        assert_eq!(template.region_count(), 0);

        let template = template.with_control_region(ControlRegion::point("test", Point3::origin()));
        assert_eq!(template.region_count(), 1);
    }

    #[test]
    fn test_control_regions() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh)
            .with_control_region(ControlRegion::point("test", Point3::origin()));

        let regions = template.control_regions();
        assert_eq!(regions.len(), 1);
        assert!(regions.contains_key("test"));
    }

    #[test]
    fn test_fit_empty_mesh() {
        let mesh = IndexedMesh::new();
        let template = FitTemplate::new(mesh);

        let result = template.fit(&FitParams::new());
        assert!(result.is_err());
    }

    #[test]
    fn test_fit_no_constraints() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh);

        let result = template.fit(&FitParams::new());
        assert!(result.is_err());
    }

    #[test]
    fn test_fit_to_measurements() {
        let mesh = make_triangle();
        let template = FitTemplate::new(mesh);

        let measurements = vec![("test".to_string(), Measurement::exact(100.0))];

        // This will fail because we don't have matching measurement regions
        // but it exercises the code path
        let result = template.fit_to_measurements(measurements);
        // Result depends on whether measurement region exists
        let _ = result;
    }
}
