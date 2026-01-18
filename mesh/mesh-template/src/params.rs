//! Fitting parameters for template customization.

use crate::Measurement;
use mesh_types::IndexedMesh;
use std::collections::HashMap;
use nalgebra::Point3;

/// Parameters for fitting a template to targets.
///
/// `FitParams` controls the fitting process through three types of constraints:
///
/// 1. **Target scan** - A 3D mesh to align the template to
/// 2. **Landmark targets** - Specific positions for named control regions
/// 3. **Measurement targets** - Dimensional constraints for measurement regions
///
/// # Examples
///
/// ## Basic landmark fitting
///
/// ```
/// use mesh_template::FitParams;
/// use nalgebra::Point3;
///
/// let params = FitParams::new()
///     .with_landmark_target("nose_tip", Point3::new(0.0, 0.0, 10.0))
///     .with_landmark_target("chin", Point3::new(0.0, 0.0, -5.0));
/// ```
///
/// ## Scan fitting with refinement
///
/// ```
/// use mesh_template::FitParams;
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut scan = IndexedMesh::new();
/// scan.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// // ... add more vertices and faces
///
/// let params = FitParams::new()
///     .with_target_scan(scan)
///     .with_registration_iterations(200)
///     .with_smoothness(0.5);
/// ```
///
/// ## Measurement-based fitting
///
/// ```
/// use mesh_template::{FitParams, Measurement};
///
/// let params = FitParams::new()
///     .with_measurement("waist", Measurement::exact(80.0))
///     .with_measurement("chest", Measurement::with_tolerance(100.0, 5.0));
/// ```
#[derive(Debug, Clone)]
pub struct FitParams {
    /// Target scan mesh for alignment.
    ///
    /// If provided, the template will be rigidly aligned to this scan
    /// using ICP registration before deformation.
    pub target_scan: Option<IndexedMesh>,

    /// Target positions for named landmarks.
    ///
    /// Each entry maps a control region name to its desired position.
    pub landmark_targets: HashMap<String, Point3<f64>>,

    /// Target measurements for named measurement regions.
    ///
    /// Each entry maps a measurement region name to its constraint.
    pub measurement_targets: HashMap<String, Measurement>,

    /// Smoothness parameter for morphing (default: 1.0).
    ///
    /// Higher values produce smoother deformations but may not match
    /// constraints as precisely. Lower values match constraints more
    /// closely but may produce less smooth results.
    pub smoothness: f64,

    /// Number of iterations for ICP registration (default: 100).
    pub registration_iterations: usize,

    /// Convergence threshold for registration (default: 1e-6).
    pub convergence_threshold: f64,
}

impl FitParams {
    /// Default smoothness value.
    pub const DEFAULT_SMOOTHNESS: f64 = 1.0;

    /// Default number of registration iterations.
    pub const DEFAULT_REGISTRATION_ITERATIONS: usize = 100;

    /// Default convergence threshold.
    pub const DEFAULT_CONVERGENCE_THRESHOLD: f64 = 1e-6;

    /// Creates new fitting parameters with default values.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::FitParams;
    ///
    /// let params = FitParams::new();
    /// assert!(params.target_scan.is_none());
    /// assert!(params.landmark_targets.is_empty());
    /// ```
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Sets the target scan mesh for alignment.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::FitParams;
    /// use mesh_types::{IndexedMesh, Vertex};
    ///
    /// let mut scan = IndexedMesh::new();
    /// scan.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
    ///
    /// let params = FitParams::new().with_target_scan(scan);
    /// assert!(params.target_scan.is_some());
    /// ```
    #[must_use]
    pub fn with_target_scan(mut self, scan: IndexedMesh) -> Self {
        self.target_scan = Some(scan);
        self
    }

    /// Adds a single landmark target.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::FitParams;
    /// use nalgebra::Point3;
    ///
    /// let params = FitParams::new()
    ///     .with_landmark_target("tip", Point3::new(0.0, 0.0, 10.0));
    /// assert!(params.landmark_targets.contains_key("tip"));
    /// ```
    #[must_use]
    pub fn with_landmark_target(mut self, name: impl Into<String>, target: Point3<f64>) -> Self {
        self.landmark_targets.insert(name.into(), target);
        self
    }

    /// Adds multiple landmark targets.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::FitParams;
    /// use nalgebra::Point3;
    /// use std::collections::HashMap;
    ///
    /// let mut targets = HashMap::new();
    /// targets.insert("a".to_string(), Point3::new(0.0, 0.0, 0.0));
    /// targets.insert("b".to_string(), Point3::new(1.0, 0.0, 0.0));
    ///
    /// let params = FitParams::new().with_landmark_targets(targets);
    /// assert_eq!(params.landmark_targets.len(), 2);
    /// ```
    #[must_use]
    pub fn with_landmark_targets(mut self, targets: impl IntoIterator<Item = (String, Point3<f64>)>) -> Self {
        for (name, target) in targets {
            self.landmark_targets.insert(name, target);
        }
        self
    }

    /// Adds a single measurement target.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::{FitParams, Measurement};
    ///
    /// let params = FitParams::new()
    ///     .with_measurement("waist", Measurement::exact(80.0));
    /// assert!(params.measurement_targets.contains_key("waist"));
    /// ```
    #[must_use]
    pub fn with_measurement(mut self, name: impl Into<String>, measurement: Measurement) -> Self {
        self.measurement_targets.insert(name.into(), measurement);
        self
    }

    /// Adds multiple measurement targets.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::{FitParams, Measurement};
    /// use std::collections::HashMap;
    ///
    /// let mut measurements = HashMap::new();
    /// measurements.insert("waist".to_string(), Measurement::exact(80.0));
    /// measurements.insert("chest".to_string(), Measurement::exact(100.0));
    ///
    /// let params = FitParams::new().with_measurements(measurements);
    /// assert_eq!(params.measurement_targets.len(), 2);
    /// ```
    #[must_use]
    pub fn with_measurements(mut self, measurements: impl IntoIterator<Item = (String, Measurement)>) -> Self {
        for (name, measurement) in measurements {
            self.measurement_targets.insert(name, measurement);
        }
        self
    }

    /// Sets the smoothness parameter for morphing.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::FitParams;
    ///
    /// let params = FitParams::new().with_smoothness(0.5);
    /// assert!((params.smoothness - 0.5).abs() < 1e-10);
    /// ```
    #[must_use]
    pub const fn with_smoothness(mut self, smoothness: f64) -> Self {
        self.smoothness = smoothness;
        self
    }

    /// Sets the number of ICP registration iterations.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::FitParams;
    ///
    /// let params = FitParams::new().with_registration_iterations(200);
    /// assert_eq!(params.registration_iterations, 200);
    /// ```
    #[must_use]
    pub const fn with_registration_iterations(mut self, iterations: usize) -> Self {
        self.registration_iterations = iterations;
        self
    }

    /// Sets the convergence threshold for registration.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_template::FitParams;
    ///
    /// let params = FitParams::new().with_convergence_threshold(1e-8);
    /// assert!((params.convergence_threshold - 1e-8).abs() < 1e-15);
    /// ```
    #[must_use]
    pub const fn with_convergence_threshold(mut self, threshold: f64) -> Self {
        self.convergence_threshold = threshold;
        self
    }

    /// Returns true if any constraints are defined.
    #[must_use]
    pub fn has_constraints(&self) -> bool {
        self.target_scan.is_some()
            || !self.landmark_targets.is_empty()
            || !self.measurement_targets.is_empty()
    }
}

impl Default for FitParams {
    fn default() -> Self {
        Self {
            target_scan: None,
            landmark_targets: HashMap::new(),
            measurement_targets: HashMap::new(),
            smoothness: Self::DEFAULT_SMOOTHNESS,
            registration_iterations: Self::DEFAULT_REGISTRATION_ITERATIONS,
            convergence_threshold: Self::DEFAULT_CONVERGENCE_THRESHOLD,
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    #[test]
    fn test_default() {
        let params = FitParams::default();
        assert!(params.target_scan.is_none());
        assert!(params.landmark_targets.is_empty());
        assert!(params.measurement_targets.is_empty());
        assert_relative_eq!(params.smoothness, 1.0);
        assert_eq!(params.registration_iterations, 100);
    }

    #[test]
    fn test_new() {
        let params = FitParams::new();
        assert!(params.target_scan.is_none());
    }

    #[test]
    fn test_with_target_scan() {
        let mut scan = IndexedMesh::new();
        scan.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));

        let params = FitParams::new().with_target_scan(scan);
        assert!(params.target_scan.is_some());
        assert_eq!(params.target_scan.unwrap().vertices.len(), 1);
    }

    #[test]
    fn test_with_landmark_target() {
        let params = FitParams::new()
            .with_landmark_target("a", Point3::new(1.0, 2.0, 3.0))
            .with_landmark_target("b", Point3::new(4.0, 5.0, 6.0));

        assert_eq!(params.landmark_targets.len(), 2);

        let a = params.landmark_targets.get("a").unwrap();
        assert_relative_eq!(a.x, 1.0);
    }

    #[test]
    fn test_with_landmark_targets() {
        let targets = vec![
            ("a".to_string(), Point3::new(1.0, 0.0, 0.0)),
            ("b".to_string(), Point3::new(0.0, 1.0, 0.0)),
            ("c".to_string(), Point3::new(0.0, 0.0, 1.0)),
        ];

        let params = FitParams::new().with_landmark_targets(targets);
        assert_eq!(params.landmark_targets.len(), 3);
    }

    #[test]
    fn test_with_measurement() {
        let params = FitParams::new()
            .with_measurement("waist", Measurement::exact(80.0));

        assert_eq!(params.measurement_targets.len(), 1);
        let m = params.measurement_targets.get("waist").unwrap();
        assert_relative_eq!(m.value, 80.0);
    }

    #[test]
    fn test_with_measurements() {
        let measurements = vec![
            ("waist".to_string(), Measurement::exact(80.0)),
            ("chest".to_string(), Measurement::exact(100.0)),
        ];

        let params = FitParams::new().with_measurements(measurements);
        assert_eq!(params.measurement_targets.len(), 2);
    }

    #[test]
    fn test_with_smoothness() {
        let params = FitParams::new().with_smoothness(0.5);
        assert_relative_eq!(params.smoothness, 0.5);
    }

    #[test]
    fn test_with_registration_iterations() {
        let params = FitParams::new().with_registration_iterations(50);
        assert_eq!(params.registration_iterations, 50);
    }

    #[test]
    fn test_with_convergence_threshold() {
        let params = FitParams::new().with_convergence_threshold(1e-8);
        assert_relative_eq!(params.convergence_threshold, 1e-8);
    }

    #[test]
    fn test_has_constraints() {
        let params = FitParams::new();
        assert!(!params.has_constraints());

        let params = FitParams::new().with_landmark_target("a", Point3::origin());
        assert!(params.has_constraints());

        let params = FitParams::new().with_measurement("m", Measurement::exact(100.0));
        assert!(params.has_constraints());

        let mut scan = IndexedMesh::new();
        scan.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        let params = FitParams::new().with_target_scan(scan);
        assert!(params.has_constraints());
    }

    #[test]
    fn test_builder_chaining() {
        let mut scan = IndexedMesh::new();
        scan.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));

        let params = FitParams::new()
            .with_target_scan(scan)
            .with_landmark_target("a", Point3::origin())
            .with_measurement("m", Measurement::exact(100.0))
            .with_smoothness(0.8)
            .with_registration_iterations(150)
            .with_convergence_threshold(1e-7);

        assert!(params.target_scan.is_some());
        assert_eq!(params.landmark_targets.len(), 1);
        assert_eq!(params.measurement_targets.len(), 1);
        assert_relative_eq!(params.smoothness, 0.8);
        assert_eq!(params.registration_iterations, 150);
        assert_relative_eq!(params.convergence_threshold, 1e-7);
    }
}
