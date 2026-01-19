//! Morphing parameters and configuration.
//!
//! This module provides the [`MorphParams`] struct for configuring mesh
//! deformation operations.

use crate::{Constraint, FfdConfig, RbfKernel};
use std::collections::HashSet;

/// The morphing algorithm to use.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum MorphAlgorithm {
    /// Radial Basis Function morphing.
    ///
    /// RBF provides smooth, global deformation controlled by constraint points.
    /// Good for natural-looking deformations where constraints should influence
    /// the entire mesh smoothly.
    Rbf(RbfKernel),

    /// Free-Form Deformation.
    ///
    /// FFD deforms the mesh using a control lattice with Bernstein polynomial
    /// interpolation. Good for topology-preserving, large-scale deformations.
    Ffd(FfdConfig),
}

impl Default for MorphAlgorithm {
    fn default() -> Self {
        Self::Rbf(RbfKernel::default())
    }
}

/// Parameters for mesh morphing.
///
/// Use the builder methods to configure the morph operation.
///
/// # Examples
///
/// ## RBF Morphing with Thin-Plate Splines
///
/// ```
/// use mesh_morph::{MorphParams, Constraint};
/// use nalgebra::Point3;
///
/// let params = MorphParams::rbf()
///     .with_constraints(vec![
///         Constraint::point(
///             Point3::new(0.0, 0.0, 0.0),
///             Point3::new(1.0, 0.0, 0.0),
///         ),
///     ]);
/// ```
///
/// ## RBF with Gaussian Kernel
///
/// ```
/// use mesh_morph::{MorphParams, Constraint};
/// use nalgebra::{Point3, Vector3};
///
/// let params = MorphParams::rbf_gaussian(1.5)
///     .with_constraints(vec![
///         Constraint::displacement(
///             Point3::new(0.0, 0.0, 0.0),
///             Vector3::new(0.0, 0.0, 1.0),
///         ),
///     ]);
/// ```
///
/// ## FFD with Custom Resolution
///
/// ```
/// use mesh_morph::{MorphParams, Constraint};
/// use nalgebra::Point3;
///
/// let params = MorphParams::ffd_with_resolution(5, 5, 5)
///     .unwrap()
///     .with_constraints(vec![
///         Constraint::point(
///             Point3::new(0.5, 0.5, 0.5),
///             Point3::new(0.5, 0.5, 1.0),
///         ),
///     ]);
/// ```
#[derive(Debug, Clone)]
pub struct MorphParams {
    /// The morphing algorithm.
    pub algorithm: MorphAlgorithm,
    /// The constraints defining the deformation.
    pub constraints: Vec<Constraint>,
    /// Optional mask of vertex indices to deform.
    ///
    /// If `None`, all vertices are deformed.
    pub vertex_mask: Option<HashSet<usize>>,
}

impl Default for MorphParams {
    fn default() -> Self {
        Self::rbf()
    }
}

impl MorphParams {
    /// Creates parameters for RBF morphing with thin-plate spline kernel.
    ///
    /// This is the default and most commonly used configuration for smooth,
    /// natural deformations.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::MorphParams;
    ///
    /// let params = MorphParams::rbf();
    /// ```
    #[must_use]
    pub const fn rbf() -> Self {
        Self {
            algorithm: MorphAlgorithm::Rbf(RbfKernel::ThinPlateSpline),
            constraints: Vec::new(),
            vertex_mask: None,
        }
    }

    /// Creates parameters for RBF morphing with Gaussian kernel.
    ///
    /// The Gaussian kernel provides more local deformation with controllable
    /// support radius via the sigma parameter.
    ///
    /// # Arguments
    ///
    /// * `sigma` - Controls the influence radius. Larger values spread influence further.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::MorphParams;
    ///
    /// let params = MorphParams::rbf_gaussian(1.0);
    /// ```
    #[must_use]
    pub const fn rbf_gaussian(sigma: f64) -> Self {
        Self {
            algorithm: MorphAlgorithm::Rbf(RbfKernel::Gaussian { sigma }),
            constraints: Vec::new(),
            vertex_mask: None,
        }
    }

    /// Creates parameters for RBF morphing with multiquadric kernel.
    ///
    /// The multiquadric kernel provides a good balance between local and global
    /// deformation effects.
    ///
    /// # Arguments
    ///
    /// * `c` - The shape parameter.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::MorphParams;
    ///
    /// let params = MorphParams::rbf_multiquadric(1.0);
    /// ```
    #[must_use]
    pub const fn rbf_multiquadric(c: f64) -> Self {
        Self {
            algorithm: MorphAlgorithm::Rbf(RbfKernel::Multiquadric { c }),
            constraints: Vec::new(),
            vertex_mask: None,
        }
    }

    /// Creates parameters for RBF morphing with inverse multiquadric kernel.
    ///
    /// This kernel provides strong local effects with rapid falloff.
    ///
    /// # Arguments
    ///
    /// * `c` - The shape parameter.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::MorphParams;
    ///
    /// let params = MorphParams::rbf_inverse_multiquadric(1.0);
    /// ```
    #[must_use]
    pub const fn rbf_inverse_multiquadric(c: f64) -> Self {
        Self {
            algorithm: MorphAlgorithm::Rbf(RbfKernel::InverseMultiquadric { c }),
            constraints: Vec::new(),
            vertex_mask: None,
        }
    }

    /// Creates parameters for FFD with default 4×4×4 lattice resolution.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::MorphParams;
    ///
    /// let params = MorphParams::ffd();
    /// ```
    #[must_use]
    pub fn ffd() -> Self {
        Self {
            algorithm: MorphAlgorithm::Ffd(FfdConfig::default()),
            constraints: Vec::new(),
            vertex_mask: None,
        }
    }

    /// Creates parameters for FFD with specified lattice resolution.
    ///
    /// # Arguments
    ///
    /// * `nx` - Control points in X direction
    /// * `ny` - Control points in Y direction
    /// * `nz` - Control points in Z direction
    ///
    /// # Errors
    ///
    /// Returns an error if any dimension is less than 2.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::MorphParams;
    ///
    /// let params = MorphParams::ffd_with_resolution(5, 5, 5).unwrap();
    /// ```
    pub fn ffd_with_resolution(nx: usize, ny: usize, nz: usize) -> crate::MorphResult<Self> {
        let config = FfdConfig::new(nx, ny, nz)?;
        Ok(Self {
            algorithm: MorphAlgorithm::Ffd(config),
            constraints: Vec::new(),
            vertex_mask: None,
        })
    }

    /// Sets the constraints for the morphing operation.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::{MorphParams, Constraint};
    /// use nalgebra::Point3;
    ///
    /// let params = MorphParams::rbf()
    ///     .with_constraints(vec![
    ///         Constraint::point(
    ///             Point3::new(0.0, 0.0, 0.0),
    ///             Point3::new(1.0, 0.0, 0.0),
    ///         ),
    ///     ]);
    ///
    /// assert_eq!(params.constraints.len(), 1);
    /// ```
    #[must_use]
    pub fn with_constraints(mut self, constraints: Vec<Constraint>) -> Self {
        self.constraints = constraints;
        self
    }

    /// Adds a single constraint.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::{MorphParams, Constraint};
    /// use nalgebra::Point3;
    ///
    /// let params = MorphParams::rbf()
    ///     .with_constraint(Constraint::point(
    ///         Point3::new(0.0, 0.0, 0.0),
    ///         Point3::new(1.0, 0.0, 0.0),
    ///     ))
    ///     .with_constraint(Constraint::point(
    ///         Point3::new(1.0, 0.0, 0.0),
    ///         Point3::new(2.0, 0.0, 0.0),
    ///     ));
    ///
    /// assert_eq!(params.constraints.len(), 2);
    /// ```
    #[must_use]
    pub fn with_constraint(mut self, constraint: Constraint) -> Self {
        self.constraints.push(constraint);
        self
    }

    /// Sets the vertex mask for selective deformation.
    ///
    /// Only vertices with indices in the mask will be deformed.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::MorphParams;
    /// use std::collections::HashSet;
    ///
    /// let mask: HashSet<usize> = [0, 1, 2, 3].into_iter().collect();
    /// let params = MorphParams::rbf()
    ///     .with_vertex_mask(mask);
    ///
    /// assert!(params.vertex_mask.is_some());
    /// ```
    #[must_use]
    pub fn with_vertex_mask(mut self, mask: HashSet<usize>) -> Self {
        self.vertex_mask = Some(mask);
        self
    }

    /// Clears the vertex mask so all vertices are deformed.
    #[must_use]
    pub fn without_vertex_mask(mut self) -> Self {
        self.vertex_mask = None;
        self
    }

    /// Returns whether a vertex index should be deformed.
    #[must_use]
    pub fn should_deform_vertex(&self, index: usize) -> bool {
        self.vertex_mask
            .as_ref()
            .is_none_or(|mask| mask.contains(&index))
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::needless_range_loop
)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_default_params() {
        let params = MorphParams::default();
        assert!(matches!(
            params.algorithm,
            MorphAlgorithm::Rbf(RbfKernel::ThinPlateSpline)
        ));
        assert!(params.constraints.is_empty());
        assert!(params.vertex_mask.is_none());
    }

    #[test]
    fn test_rbf_builders() {
        let tps = MorphParams::rbf();
        assert!(matches!(
            tps.algorithm,
            MorphAlgorithm::Rbf(RbfKernel::ThinPlateSpline)
        ));

        let gaussian = MorphParams::rbf_gaussian(2.0);
        assert!(matches!(
            gaussian.algorithm,
            MorphAlgorithm::Rbf(RbfKernel::Gaussian { sigma }) if (sigma - 2.0).abs() < f64::EPSILON
        ));

        let mq = MorphParams::rbf_multiquadric(1.5);
        assert!(matches!(
            mq.algorithm,
            MorphAlgorithm::Rbf(RbfKernel::Multiquadric { c }) if (c - 1.5).abs() < f64::EPSILON
        ));

        let imq = MorphParams::rbf_inverse_multiquadric(0.5);
        assert!(matches!(
            imq.algorithm,
            MorphAlgorithm::Rbf(RbfKernel::InverseMultiquadric { c }) if (c - 0.5).abs() < f64::EPSILON
        ));
    }

    #[test]
    fn test_ffd_builders() {
        let ffd = MorphParams::ffd();
        if let MorphAlgorithm::Ffd(config) = ffd.algorithm {
            assert_eq!(config.resolution_x, 4);
            assert_eq!(config.resolution_y, 4);
            assert_eq!(config.resolution_z, 4);
        } else {
            panic!("Expected FFD algorithm");
        }

        let ffd_custom = MorphParams::ffd_with_resolution(5, 6, 7).unwrap();
        if let MorphAlgorithm::Ffd(config) = ffd_custom.algorithm {
            assert_eq!(config.resolution_x, 5);
            assert_eq!(config.resolution_y, 6);
            assert_eq!(config.resolution_z, 7);
        } else {
            panic!("Expected FFD algorithm");
        }
    }

    #[test]
    fn test_ffd_invalid_resolution() {
        assert!(MorphParams::ffd_with_resolution(1, 4, 4).is_err());
        assert!(MorphParams::ffd_with_resolution(4, 1, 4).is_err());
        assert!(MorphParams::ffd_with_resolution(4, 4, 1).is_err());
    }

    #[test]
    fn test_with_constraints() {
        let params = MorphParams::rbf()
            .with_constraints(vec![Constraint::point(
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
            )])
            .with_constraint(Constraint::point(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
            ));

        assert_eq!(params.constraints.len(), 2);
    }

    #[test]
    fn test_vertex_mask() {
        let mask: HashSet<usize> = [0, 1, 2].into_iter().collect();
        let params = MorphParams::rbf().with_vertex_mask(mask);

        assert!(params.should_deform_vertex(0));
        assert!(params.should_deform_vertex(1));
        assert!(params.should_deform_vertex(2));
        assert!(!params.should_deform_vertex(3));
        assert!(!params.should_deform_vertex(100));
    }

    #[test]
    fn test_without_vertex_mask() {
        let mask: HashSet<usize> = [0, 1].into_iter().collect();
        let params = MorphParams::rbf()
            .with_vertex_mask(mask)
            .without_vertex_mask();

        assert!(params.vertex_mask.is_none());
        assert!(params.should_deform_vertex(0));
        assert!(params.should_deform_vertex(100));
    }
}
