//! Radial Basis Function (RBF) morphing.
//!
//! RBF morphing provides smooth, global deformation controlled by constraint points.
//! The deformation field is constructed by solving a linear system that interpolates
//! the constraint displacements.
//!
//! # Supported Kernels
//!
//! - **Thin-Plate Spline (TPS)**: Produces smooth, natural deformations
//! - **Gaussian**: Provides local deformation with controllable support
//! - **Multiquadric**: Good balance between local and global effects
//! - **Inverse Multiquadric**: Strong local effect with rapid falloff

use crate::{Constraint, MorphError, MorphResult};
use nalgebra::{DMatrix, DVector, Point3, Vector3};

/// RBF kernel function type.
///
/// Different kernels produce different deformation characteristics.
///
/// # Examples
///
/// ```
/// use mesh_morph::RbfKernel;
///
/// // Thin-plate spline - smooth, natural deformations
/// let tps = RbfKernel::ThinPlateSpline;
///
/// // Gaussian - local effect with sigma parameter
/// let gaussian = RbfKernel::Gaussian { sigma: 1.0 };
///
/// // Multiquadric - balance between local and global
/// let mq = RbfKernel::Multiquadric { c: 1.0 };
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
#[derive(Default)]
pub enum RbfKernel {
    /// Thin-plate spline: r² log(r)
    ///
    /// Produces smooth, natural deformations. This is the default and most
    /// commonly used kernel for mesh morphing.
    #[default]
    ThinPlateSpline,

    /// Gaussian: exp(-r²/σ²)
    ///
    /// Provides local deformation with controllable support radius.
    /// Larger sigma values spread the influence further.
    Gaussian {
        /// The sigma parameter controlling the support radius.
        sigma: f64,
    },

    /// Multiquadric: √(r² + c²)
    ///
    /// Good balance between local and global effects.
    Multiquadric {
        /// The shape parameter.
        c: f64,
    },

    /// Inverse Multiquadric: 1/√(r² + c²)
    ///
    /// Strong local effect with rapid falloff.
    InverseMultiquadric {
        /// The shape parameter.
        c: f64,
    },
}

impl RbfKernel {
    /// Evaluates the kernel function at distance r.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::RbfKernel;
    ///
    /// let kernel = RbfKernel::Gaussian { sigma: 1.0 };
    /// let value = kernel.evaluate(0.0);
    /// assert!((value - 1.0).abs() < 1e-10); // Maximum at r=0
    /// ```
    #[must_use]
    pub fn evaluate(&self, r: f64) -> f64 {
        match self {
            Self::ThinPlateSpline => {
                if r < 1e-10 {
                    0.0
                } else {
                    let r2 = r * r;
                    r2 * r.ln()
                }
            }
            Self::Gaussian { sigma } => {
                let r2 = r * r;
                let s2 = sigma * sigma;
                (-r2 / s2).exp()
            }
            Self::Multiquadric { c } => {
                let r2 = r * r;
                let c2 = c * c;
                (r2 + c2).sqrt()
            }
            Self::InverseMultiquadric { c } => {
                let r2 = r * r;
                let c2 = c * c;
                1.0 / (r2 + c2).sqrt()
            }
        }
    }

    /// Returns whether this kernel requires polynomial terms for stability.
    ///
    /// Thin-plate splines require polynomial terms to handle affine
    /// transformations properly.
    #[must_use]
    pub const fn needs_polynomial_terms(&self) -> bool {
        matches!(self, Self::ThinPlateSpline)
    }
}

/// RBF interpolation solver.
///
/// This struct encapsulates the solved RBF system and can evaluate
/// the deformation field at any point.
#[derive(Debug)]
pub struct RbfInterpolator {
    /// The constraint points.
    constraints: Vec<Constraint>,
    /// The kernel function.
    kernel: RbfKernel,
    /// Weights for x displacement.
    weights_x: DVector<f64>,
    /// Weights for y displacement.
    weights_y: DVector<f64>,
    /// Weights for z displacement.
    weights_z: DVector<f64>,
    /// Whether polynomial terms are included.
    has_polynomial: bool,
}

impl RbfInterpolator {
    /// Creates a new RBF interpolator from constraints.
    ///
    /// This solves the linear system to find the interpolation weights.
    pub fn new(constraints: &[Constraint], kernel: RbfKernel) -> MorphResult<Self> {
        if constraints.is_empty() {
            return Err(MorphError::NoConstraints);
        }

        let n = constraints.len();
        let has_polynomial = kernel.needs_polynomial_terms();

        // Matrix size: n constraints + 4 polynomial terms if needed
        let size = if has_polynomial { n + 4 } else { n };

        // Build the interpolation matrix
        let mut matrix = DMatrix::<f64>::zeros(size, size);

        // Fill the kernel evaluation block
        for i in 0..n {
            for j in 0..n {
                let r = (constraints[i].source - constraints[j].source).norm();
                let k = kernel.evaluate(r) * constraints[i].weight;
                matrix[(i, j)] = k;
            }

            // Add regularization for numerical stability
            matrix[(i, i)] += 1e-8;
        }

        // Add polynomial terms for TPS
        if has_polynomial {
            for i in 0..n {
                let p = &constraints[i].source;
                // Polynomial block: [1, x, y, z]
                matrix[(i, n)] = 1.0;
                matrix[(i, n + 1)] = p.x;
                matrix[(i, n + 2)] = p.y;
                matrix[(i, n + 3)] = p.z;

                matrix[(n, i)] = 1.0;
                matrix[(n + 1, i)] = p.x;
                matrix[(n + 2, i)] = p.y;
                matrix[(n + 3, i)] = p.z;
            }
        }

        // Build right-hand side vectors (displacements)
        let mut rhs_x = DVector::<f64>::zeros(size);
        let mut rhs_y = DVector::<f64>::zeros(size);
        let mut rhs_z = DVector::<f64>::zeros(size);

        for (i, c) in constraints.iter().enumerate() {
            let disp = c.displacement_vector();
            rhs_x[i] = disp.x;
            rhs_y[i] = disp.y;
            rhs_z[i] = disp.z;
        }

        // Solve using SVD for numerical stability
        let svd = matrix.svd(true, true);

        let weights_x = svd
            .solve(&rhs_x, 1e-10)
            .map_err(|_| MorphError::DegenerateSystem("SVD solution failed for X".to_string()))?;
        let weights_y = svd
            .solve(&rhs_y, 1e-10)
            .map_err(|_| MorphError::DegenerateSystem("SVD solution failed for Y".to_string()))?;
        let weights_z = svd
            .solve(&rhs_z, 1e-10)
            .map_err(|_| MorphError::DegenerateSystem("SVD solution failed for Z".to_string()))?;

        Ok(Self {
            constraints: constraints.to_vec(),
            kernel,
            weights_x,
            weights_y,
            weights_z,
            has_polynomial,
        })
    }

    /// Evaluates the displacement at a given point.
    pub fn evaluate(&self, point: &Point3<f64>) -> Vector3<f64> {
        let n = self.constraints.len();

        let mut dx = 0.0;
        let mut dy = 0.0;
        let mut dz = 0.0;

        // RBF contribution
        for (i, c) in self.constraints.iter().enumerate() {
            let r = (point - c.source).norm();
            let k = self.kernel.evaluate(r);
            dx += self.weights_x[i] * k;
            dy += self.weights_y[i] * k;
            dz += self.weights_z[i] * k;
        }

        // Polynomial contribution
        if self.has_polynomial {
            // Constant term
            dx += self.weights_x[n];
            dy += self.weights_y[n];
            dz += self.weights_z[n];

            // Linear terms
            dx += self.weights_x[n + 1] * point.x;
            dy += self.weights_y[n + 1] * point.x;
            dz += self.weights_z[n + 1] * point.x;

            dx += self.weights_x[n + 2] * point.y;
            dy += self.weights_y[n + 2] * point.y;
            dz += self.weights_z[n + 2] * point.y;

            dx += self.weights_x[n + 3] * point.z;
            dy += self.weights_y[n + 3] * point.z;
            dz += self.weights_z[n + 3] * point.z;
        }

        Vector3::new(dx, dy, dz)
    }

    /// Transforms a point using the RBF deformation field.
    pub fn transform(&self, point: &Point3<f64>) -> Point3<f64> {
        point + self.evaluate(point)
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
    use approx::assert_relative_eq;

    #[test]
    fn test_tps_kernel_at_zero() {
        let kernel = RbfKernel::ThinPlateSpline;
        assert!((kernel.evaluate(0.0)).abs() < 1e-10);
    }

    #[test]
    fn test_tps_kernel_nonzero() {
        let kernel = RbfKernel::ThinPlateSpline;
        let r: f64 = 2.0;
        let expected = r * r * r.ln();
        assert_relative_eq!(kernel.evaluate(r), expected, epsilon = 1e-10);
    }

    #[test]
    fn test_gaussian_kernel_at_zero() {
        let kernel = RbfKernel::Gaussian { sigma: 1.0 };
        assert_relative_eq!(kernel.evaluate(0.0), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_gaussian_kernel_decay() {
        let kernel = RbfKernel::Gaussian { sigma: 1.0 };
        // At r = sigma, value should be exp(-1)
        assert_relative_eq!(kernel.evaluate(1.0), (-1.0_f64).exp(), epsilon = 1e-10);
    }

    #[test]
    fn test_multiquadric_kernel() {
        let kernel = RbfKernel::Multiquadric { c: 1.0 };
        // At r = 0, value should be c
        assert_relative_eq!(kernel.evaluate(0.0), 1.0, epsilon = 1e-10);
        // At r = 1, c = 1: sqrt(1 + 1) = sqrt(2)
        assert_relative_eq!(kernel.evaluate(1.0), 2.0_f64.sqrt(), epsilon = 1e-10);
    }

    #[test]
    fn test_inverse_multiquadric_kernel() {
        let kernel = RbfKernel::InverseMultiquadric { c: 1.0 };
        // At r = 0, value should be 1/c
        assert_relative_eq!(kernel.evaluate(0.0), 1.0, epsilon = 1e-10);
        // At r = 1, c = 1: 1/sqrt(2)
        assert_relative_eq!(kernel.evaluate(1.0), 1.0 / 2.0_f64.sqrt(), epsilon = 1e-10);
    }

    #[test]
    fn test_rbf_interpolator_identity() {
        // Constraints that map points to themselves
        let constraints = vec![
            Constraint::point(Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
            Constraint::point(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)),
            Constraint::point(Point3::new(0.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)),
        ];

        let interp = RbfInterpolator::new(&constraints, RbfKernel::ThinPlateSpline).unwrap();

        // Displacement should be zero at constraint points
        for c in &constraints {
            let disp = interp.evaluate(&c.source);
            assert!(
                disp.norm() < 1e-6,
                "Expected zero displacement, got {:?}",
                disp
            );
        }
    }

    #[test]
    fn test_rbf_interpolator_translation() {
        let offset = Vector3::new(1.0, 2.0, 3.0);
        let constraints = vec![
            Constraint::point(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0) + offset,
            ),
            Constraint::point(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0) + offset,
            ),
            Constraint::point(
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0) + offset,
            ),
            Constraint::point(
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(0.0, 0.0, 1.0) + offset,
            ),
        ];

        let interp = RbfInterpolator::new(&constraints, RbfKernel::ThinPlateSpline).unwrap();

        // Check at constraint points
        for c in &constraints {
            let result = interp.transform(&c.source);
            assert_relative_eq!(result.x, c.target.x, epsilon = 1e-4);
            assert_relative_eq!(result.y, c.target.y, epsilon = 1e-4);
            assert_relative_eq!(result.z, c.target.z, epsilon = 1e-4);
        }
    }

    #[test]
    fn test_rbf_interpolator_gaussian() {
        let constraints = vec![
            Constraint::point(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)),
            Constraint::point(Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
        ];

        let interp =
            RbfInterpolator::new(&constraints, RbfKernel::Gaussian { sigma: 1.0 }).unwrap();

        // At constraint points, should get expected displacement
        let p0 = interp.transform(&Point3::new(0.0, 0.0, 0.0));
        assert!(
            (p0.x - 1.0).abs() < 0.5,
            "Expected x near 1.0, got {}",
            p0.x
        );

        // Far from constraints, should have less effect
        let far = interp.evaluate(&Point3::new(10.0, 10.0, 10.0));
        assert!(far.norm() < 0.1, "Expected small displacement far away");
    }

    #[test]
    fn test_empty_constraints_error() {
        let result = RbfInterpolator::new(&[], RbfKernel::ThinPlateSpline);
        assert!(matches!(result, Err(MorphError::NoConstraints)));
    }
}
