//! Lattice generation parameters.

use crate::density::DensityMap;
use crate::types::LatticeType;

/// Configuration parameters for lattice generation.
///
/// Use the builder methods to configure the lattice, or use one of the
/// preset constructors like [`LatticeParams::cubic`] or [`LatticeParams::gyroid`].
///
/// # Examples
///
/// ```
/// use mesh_lattice::LatticeParams;
///
/// // Simple cubic lattice with 5mm cells
/// let params = LatticeParams::cubic(5.0);
///
/// // Gyroid with custom density
/// let params = LatticeParams::gyroid(8.0)
///     .with_density(0.25)
///     .with_wall_thickness(0.6);
/// ```
#[derive(Debug, Clone)]
pub struct LatticeParams {
    /// Type of lattice structure to generate.
    pub lattice_type: LatticeType,

    /// Unit cell size in mm.
    ///
    /// For strut-based lattices, this is the distance between grid nodes.
    /// For TPMS lattices, this is the period of the implicit surface.
    pub cell_size: f64,

    /// Strut thickness in mm (for strut-based lattices).
    ///
    /// This controls the diameter of cylindrical struts.
    pub strut_thickness: f64,

    /// Wall thickness in mm (for TPMS lattices).
    ///
    /// This controls the thickness of the implicit surface.
    pub wall_thickness: f64,

    /// Target density (volume fraction) from 0.0 to 1.0.
    ///
    /// Higher density = more material = stronger but heavier.
    pub density: f64,

    /// Optional density map for variable density.
    ///
    /// When set, overrides the uniform `density` field.
    pub density_map: Option<DensityMap>,

    /// Minimum feature size in mm.
    ///
    /// Features smaller than this may be removed or simplified.
    pub min_feature_size: f64,

    /// Resolution (samples per cell) for TPMS surfaces.
    ///
    /// Higher values produce smoother surfaces but more triangles.
    /// Minimum is 2, recommended is 10-15.
    pub resolution: usize,

    /// Whether to trim the lattice to the bounding box.
    ///
    /// If true, struts extending outside bounds are clipped.
    pub trim_to_bounds: bool,

    /// Whether to preserve beam data for 3MF export.
    ///
    /// When enabled, the raw beam definitions are stored in
    /// [`LatticeResult::beam_data`](crate::LatticeResult::beam_data).
    pub preserve_beam_data: bool,
}

impl Default for LatticeParams {
    fn default() -> Self {
        Self {
            lattice_type: LatticeType::Cubic,
            cell_size: 5.0,
            strut_thickness: 0.8,
            wall_thickness: 0.5,
            density: 0.3,
            density_map: None,
            min_feature_size: 0.1,
            resolution: 10,
            trim_to_bounds: true,
            preserve_beam_data: false,
        }
    }
}

impl LatticeParams {
    /// Creates a new `LatticeParams` with default values.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::LatticeParams;
    ///
    /// let params = LatticeParams::new();
    /// assert_eq!(params.cell_size, 5.0);
    /// ```
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates parameters for a cubic lattice.
    ///
    /// # Arguments
    ///
    /// * `cell_size` - Size of each unit cell in mm
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::{LatticeParams, LatticeType};
    ///
    /// let params = LatticeParams::cubic(5.0);
    /// assert_eq!(params.lattice_type, LatticeType::Cubic);
    /// assert_eq!(params.cell_size, 5.0);
    /// ```
    #[must_use]
    pub fn cubic(cell_size: f64) -> Self {
        Self {
            lattice_type: LatticeType::Cubic,
            cell_size,
            ..Default::default()
        }
    }

    /// Creates parameters for an octet-truss lattice.
    ///
    /// Octet-truss has excellent strength-to-weight ratio.
    ///
    /// # Arguments
    ///
    /// * `cell_size` - Size of each unit cell in mm
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::{LatticeParams, LatticeType};
    ///
    /// let params = LatticeParams::octet_truss(6.0);
    /// assert_eq!(params.lattice_type, LatticeType::OctetTruss);
    /// ```
    #[must_use]
    pub fn octet_truss(cell_size: f64) -> Self {
        Self {
            lattice_type: LatticeType::OctetTruss,
            cell_size,
            ..Default::default()
        }
    }

    /// Creates parameters for a gyroid TPMS lattice.
    ///
    /// Gyroid is self-supporting and excellent for FDM printing.
    /// Uses higher resolution (15) for smooth surfaces.
    ///
    /// # Arguments
    ///
    /// * `cell_size` - Period of the gyroid surface in mm
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::{LatticeParams, LatticeType};
    ///
    /// let params = LatticeParams::gyroid(8.0);
    /// assert_eq!(params.lattice_type, LatticeType::Gyroid);
    /// assert_eq!(params.resolution, 15);
    /// ```
    #[must_use]
    pub fn gyroid(cell_size: f64) -> Self {
        Self {
            lattice_type: LatticeType::Gyroid,
            cell_size,
            resolution: 15,
            ..Default::default()
        }
    }

    /// Creates parameters for a Schwarz-P TPMS lattice.
    ///
    /// # Arguments
    ///
    /// * `cell_size` - Period of the surface in mm
    #[must_use]
    pub fn schwarz_p(cell_size: f64) -> Self {
        Self {
            lattice_type: LatticeType::SchwarzP,
            cell_size,
            resolution: 15,
            ..Default::default()
        }
    }

    /// Creates parameters for a diamond TPMS lattice.
    ///
    /// # Arguments
    ///
    /// * `cell_size` - Period of the surface in mm
    #[must_use]
    pub fn diamond(cell_size: f64) -> Self {
        Self {
            lattice_type: LatticeType::Diamond,
            cell_size,
            resolution: 15,
            ..Default::default()
        }
    }

    /// Creates parameters for a Voronoi-based lattice.
    ///
    /// Note: Currently implemented as perturbed cubic (simplified).
    ///
    /// # Arguments
    ///
    /// * `cell_size` - Approximate cell size in mm
    #[must_use]
    pub fn voronoi(cell_size: f64) -> Self {
        Self {
            lattice_type: LatticeType::Voronoi,
            cell_size,
            ..Default::default()
        }
    }

    /// Sets the lattice type.
    #[must_use]
    pub const fn with_lattice_type(mut self, lattice_type: LatticeType) -> Self {
        self.lattice_type = lattice_type;
        self
    }

    /// Sets the cell size.
    #[must_use]
    pub const fn with_cell_size(mut self, cell_size: f64) -> Self {
        self.cell_size = cell_size;
        self
    }

    /// Sets the strut thickness (for strut-based lattices).
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::LatticeParams;
    ///
    /// let params = LatticeParams::cubic(5.0).with_strut_thickness(1.0);
    /// assert_eq!(params.strut_thickness, 1.0);
    /// ```
    #[must_use]
    pub const fn with_strut_thickness(mut self, thickness: f64) -> Self {
        self.strut_thickness = thickness;
        self
    }

    /// Sets the wall thickness (for TPMS lattices).
    #[must_use]
    pub const fn with_wall_thickness(mut self, thickness: f64) -> Self {
        self.wall_thickness = thickness;
        self
    }

    /// Sets the target density (volume fraction).
    ///
    /// The value is clamped to [0.0, 1.0].
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::LatticeParams;
    ///
    /// let params = LatticeParams::cubic(5.0).with_density(0.2);
    /// assert!((params.density - 0.2).abs() < 0.001);
    /// ```
    #[must_use]
    pub const fn with_density(mut self, density: f64) -> Self {
        self.density = density.clamp(0.0, 1.0);
        self
    }

    /// Sets a density map for variable density.
    ///
    /// When set, this overrides the uniform density value.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::{LatticeParams, DensityMap};
    ///
    /// let params = LatticeParams::gyroid(8.0)
    ///     .with_density_map(DensityMap::Uniform(0.3));
    /// assert!(params.density_map.is_some());
    /// ```
    #[must_use]
    pub fn with_density_map(mut self, map: DensityMap) -> Self {
        self.density_map = Some(map);
        self
    }

    /// Sets the minimum feature size.
    #[must_use]
    pub const fn with_min_feature_size(mut self, size: f64) -> Self {
        self.min_feature_size = size;
        self
    }

    /// Sets the resolution for TPMS surfaces.
    ///
    /// Minimum is 2, values below are clamped.
    #[must_use]
    pub fn with_resolution(mut self, resolution: usize) -> Self {
        self.resolution = resolution.max(2);
        self
    }

    /// Sets whether to trim the lattice to bounds.
    #[must_use]
    pub const fn with_trim_to_bounds(mut self, trim: bool) -> Self {
        self.trim_to_bounds = trim;
        self
    }

    /// Enables or disables beam data export for 3MF.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::LatticeParams;
    ///
    /// let params = LatticeParams::cubic(5.0).with_beam_export(true);
    /// assert!(params.preserve_beam_data);
    /// ```
    #[must_use]
    pub const fn with_beam_export(mut self, enable: bool) -> Self {
        self.preserve_beam_data = enable;
        self
    }

    /// Returns the effective density at a given point.
    ///
    /// If a density map is set, evaluates it at the point.
    /// Otherwise, returns the uniform density value.
    #[must_use]
    pub fn density_at(&self, point: nalgebra::Point3<f64>) -> f64 {
        self.density_map
            .as_ref()
            .map_or(self.density, |map| map.evaluate(point))
    }

    /// Validates the parameters.
    ///
    /// Returns `Ok(())` if all parameters are valid, or an error describing
    /// the first invalid parameter found.
    ///
    /// # Errors
    ///
    /// Returns [`LatticeError`](crate::LatticeError) if any parameter is invalid.
    pub fn validate(&self) -> Result<(), crate::LatticeError> {
        if self.cell_size <= 0.0 {
            return Err(crate::LatticeError::InvalidCellSize(self.cell_size));
        }

        if self.strut_thickness <= 0.0 || self.strut_thickness >= self.cell_size {
            return Err(crate::LatticeError::InvalidStrutThickness {
                thickness: self.strut_thickness,
                cell_size: self.cell_size,
            });
        }

        if !(0.0..=1.0).contains(&self.density) {
            return Err(crate::LatticeError::InvalidDensity(self.density));
        }

        if self.resolution < 2 {
            return Err(crate::LatticeError::InvalidResolution(self.resolution));
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_params() {
        let params = LatticeParams::default();
        assert_eq!(params.lattice_type, LatticeType::Cubic);
        assert!((params.cell_size - 5.0).abs() < f64::EPSILON);
        assert!((params.strut_thickness - 0.8).abs() < f64::EPSILON);
        assert!((params.density - 0.3).abs() < f64::EPSILON);
        assert_eq!(params.resolution, 10);
        assert!(params.trim_to_bounds);
        assert!(!params.preserve_beam_data);
    }

    #[test]
    fn test_cubic_preset() {
        let params = LatticeParams::cubic(10.0);
        assert_eq!(params.lattice_type, LatticeType::Cubic);
        assert!((params.cell_size - 10.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_gyroid_preset() {
        let params = LatticeParams::gyroid(8.0);
        assert_eq!(params.lattice_type, LatticeType::Gyroid);
        assert!((params.cell_size - 8.0).abs() < f64::EPSILON);
        assert_eq!(params.resolution, 15);
    }

    #[test]
    fn test_octet_truss_preset() {
        let params = LatticeParams::octet_truss(6.0);
        assert_eq!(params.lattice_type, LatticeType::OctetTruss);
        assert!((params.cell_size - 6.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_builder_chain() {
        let params = LatticeParams::cubic(5.0)
            .with_strut_thickness(1.0)
            .with_density(0.25)
            .with_resolution(12)
            .with_beam_export(true);

        assert!((params.strut_thickness - 1.0).abs() < f64::EPSILON);
        assert!((params.density - 0.25).abs() < f64::EPSILON);
        assert_eq!(params.resolution, 12);
        assert!(params.preserve_beam_data);
    }

    #[test]
    fn test_density_clamping() {
        let params = LatticeParams::new().with_density(1.5);
        assert!((params.density - 1.0).abs() < f64::EPSILON);

        let params = LatticeParams::new().with_density(-0.5);
        assert!(params.density.abs() < f64::EPSILON);
    }

    #[test]
    fn test_resolution_clamping() {
        let params = LatticeParams::new().with_resolution(1);
        assert_eq!(params.resolution, 2);
    }

    #[test]
    fn test_validate_valid_params() {
        let params = LatticeParams::cubic(5.0);
        assert!(params.validate().is_ok());
    }

    #[test]
    fn test_validate_invalid_cell_size() {
        let params = LatticeParams::new().with_cell_size(0.0);
        assert!(matches!(
            params.validate(),
            Err(crate::LatticeError::InvalidCellSize(_))
        ));
    }

    #[test]
    fn test_validate_invalid_strut_thickness() {
        let params = LatticeParams::cubic(5.0).with_strut_thickness(6.0);
        assert!(matches!(
            params.validate(),
            Err(crate::LatticeError::InvalidStrutThickness { .. })
        ));
    }
}
