//! Free-Form Deformation (FFD) morphing.
//!
//! FFD deforms a mesh by manipulating a control lattice around it.
//! The deformation is computed using Bernstein polynomial interpolation,
//! which ensures smooth, topology-preserving transformations.
//!
//! # Overview
//!
//! 1. A regular 3D lattice of control points is placed around the mesh
//! 2. Constraints move specific control points
//! 3. Each mesh vertex is deformed based on weighted influence from all control points

use crate::{Constraint, MorphError, MorphResult};
use mesh_types::Aabb;
use nalgebra::{Point3, Vector3};

/// FFD lattice configuration.
#[derive(Debug, Clone, Copy)]
pub struct FfdConfig {
    /// Number of control points in X direction.
    pub resolution_x: usize,
    /// Number of control points in Y direction.
    pub resolution_y: usize,
    /// Number of control points in Z direction.
    pub resolution_z: usize,
    /// Padding factor around the mesh bounds (fraction of size).
    pub padding: f64,
}

impl Default for FfdConfig {
    fn default() -> Self {
        Self {
            resolution_x: 4,
            resolution_y: 4,
            resolution_z: 4,
            padding: 0.01,
        }
    }
}

impl FfdConfig {
    /// Creates a new FFD configuration with the specified resolution.
    ///
    /// # Arguments
    ///
    /// * `nx` - Control points in X
    /// * `ny` - Control points in Y
    /// * `nz` - Control points in Z
    ///
    /// # Errors
    ///
    /// Returns an error if any dimension is less than 2.
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_morph::FfdConfig;
    ///
    /// let config = FfdConfig::new(4, 4, 4).unwrap();
    /// assert_eq!(config.resolution_x, 4);
    /// ```
    pub fn new(nx: usize, ny: usize, nz: usize) -> MorphResult<Self> {
        if nx < 2 || ny < 2 || nz < 2 {
            return Err(MorphError::InvalidLatticeDimensions(
                "each dimension must be at least 2".to_string(),
            ));
        }
        Ok(Self {
            resolution_x: nx,
            resolution_y: ny,
            resolution_z: nz,
            padding: 0.01,
        })
    }

    /// Sets the padding factor.
    #[must_use]
    pub const fn with_padding(mut self, padding: f64) -> Self {
        self.padding = padding;
        self
    }

    /// Returns the total number of control points.
    #[must_use]
    pub const fn control_point_count(&self) -> usize {
        self.resolution_x * self.resolution_y * self.resolution_z
    }
}

/// FFD lattice deformer.
///
/// This struct manages the control lattice and computes deformations
/// using Bernstein polynomial interpolation.
#[derive(Debug)]
pub struct FfdLattice {
    /// The lattice configuration.
    config: FfdConfig,
    /// The bounds of the lattice (with padding).
    bounds: Aabb,
    /// Control point positions (flattened 3D array).
    control_points: Vec<Point3<f64>>,
}

impl FfdLattice {
    /// Creates a new FFD lattice around the given mesh bounds.
    #[must_use]
    pub fn new(mesh_bounds: &Aabb, config: FfdConfig) -> Self {
        // Add padding to the bounds
        let size = mesh_bounds.max - mesh_bounds.min;
        let padding = size * config.padding;
        let min = mesh_bounds.min - padding;
        let max = mesh_bounds.max + padding;
        let bounds = Aabb { min, max };

        // Create control points on a regular grid
        let nx = config.resolution_x;
        let ny = config.resolution_y;
        let nz = config.resolution_z;

        let mut control_points = Vec::with_capacity(nx * ny * nz);

        for kk in 0..nz {
            for jj in 0..ny {
                for ii in 0..nx {
                    #[allow(clippy::cast_precision_loss)]
                    let param_u = ii as f64 / (nx - 1) as f64;
                    #[allow(clippy::cast_precision_loss)]
                    let param_v = jj as f64 / (ny - 1) as f64;
                    #[allow(clippy::cast_precision_loss)]
                    let param_w = kk as f64 / (nz - 1) as f64;

                    let pt = Point3::new(
                        min.x + param_u * (max.x - min.x),
                        min.y + param_v * (max.y - min.y),
                        min.z + param_w * (max.z - min.z),
                    );
                    control_points.push(pt);
                }
            }
        }

        Self {
            config,
            bounds,
            control_points,
        }
    }

    /// Applies constraints to deform the control lattice.
    #[allow(clippy::many_single_char_names)]
    pub fn apply_constraints(&mut self, constraints: &[Constraint]) {
        let nx = self.config.resolution_x;
        let ny = self.config.resolution_y;
        let nz = self.config.resolution_z;

        for constraint in constraints {
            // Find the lattice coordinates of the constraint source
            let (param_u, param_v, param_w) = self.world_to_lattice(&constraint.source);

            // Find the nearest control point
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_precision_loss
            )]
            let idx_i = (param_u * (nx - 1) as f64).round() as usize;
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_precision_loss
            )]
            let idx_j = (param_v * (ny - 1) as f64).round() as usize;
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_precision_loss
            )]
            let idx_k = (param_w * (nz - 1) as f64).round() as usize;

            let idx_i = idx_i.min(nx - 1);
            let idx_j = idx_j.min(ny - 1);
            let idx_k = idx_k.min(nz - 1);

            // Apply displacement to the control point and its neighbors
            let displacement = constraint.displacement_vector() * constraint.weight;
            self.apply_displacement_with_falloff(idx_i, idx_j, idx_k, &displacement);
        }
    }

    /// Applies displacement to a control point with falloff to neighbors.
    fn apply_displacement_with_falloff(
        &mut self,
        center_i: usize,
        center_j: usize,
        center_k: usize,
        displacement: &Vector3<f64>,
    ) {
        let nx = self.config.resolution_x;
        let ny = self.config.resolution_y;
        let nz = self.config.resolution_z;

        // Influence radius in control point indices
        let radius = 1;

        let i_min = center_i.saturating_sub(radius);
        let i_max = (center_i + radius + 1).min(nx);
        let j_min = center_j.saturating_sub(radius);
        let j_max = (center_j + radius + 1).min(ny);
        let k_min = center_k.saturating_sub(radius);
        let k_max = (center_k + radius + 1).min(nz);

        for kk in k_min..k_max {
            for jj in j_min..j_max {
                for ii in i_min..i_max {
                    // Distance from center control point
                    #[allow(clippy::cast_precision_loss)]
                    let di = (ii as f64 - center_i as f64).abs();
                    #[allow(clippy::cast_precision_loss)]
                    let dj = (jj as f64 - center_j as f64).abs();
                    #[allow(clippy::cast_precision_loss)]
                    let dk = (kk as f64 - center_k as f64).abs();
                    let dist = dk.mul_add(dk, di.mul_add(di, dj * dj)).sqrt();

                    // Gaussian-like falloff
                    let weight = (-dist * dist).exp();

                    let idx = self.control_index(ii, jj, kk);
                    self.control_points[idx] += displacement * weight;
                }
            }
        }
    }

    /// Converts world coordinates to normalized lattice coordinates (0-1).
    fn world_to_lattice(&self, point: &Point3<f64>) -> (f64, f64, f64) {
        let size = self.bounds.max - self.bounds.min;

        let param_u = if size.x > 1e-10 {
            (point.x - self.bounds.min.x) / size.x
        } else {
            0.5
        };
        let param_v = if size.y > 1e-10 {
            (point.y - self.bounds.min.y) / size.y
        } else {
            0.5
        };
        let param_w = if size.z > 1e-10 {
            (point.z - self.bounds.min.z) / size.z
        } else {
            0.5
        };

        // Clamp to valid range
        (
            param_u.clamp(0.0, 1.0),
            param_v.clamp(0.0, 1.0),
            param_w.clamp(0.0, 1.0),
        )
    }

    /// Computes the flat index for a 3D control point index.
    const fn control_index(&self, idx_i: usize, idx_j: usize, idx_k: usize) -> usize {
        let nx = self.config.resolution_x;
        let ny = self.config.resolution_y;
        idx_i + idx_j * nx + idx_k * nx * ny
    }

    /// Transforms a point using the FFD lattice.
    #[must_use]
    pub fn transform(&self, point: &Point3<f64>) -> Point3<f64> {
        let (param_u, param_v, param_w) = self.world_to_lattice(point);
        self.evaluate_bernstein(param_u, param_v, param_w)
    }

    /// Evaluates the FFD at normalized lattice coordinates using Bernstein polynomials.
    #[allow(clippy::many_single_char_names)]
    fn evaluate_bernstein(&self, param_u: f64, param_v: f64, param_w: f64) -> Point3<f64> {
        let nx = self.config.resolution_x;
        let ny = self.config.resolution_y;
        let nz = self.config.resolution_z;

        let degree_n = nx - 1;
        let degree_m = ny - 1;
        let degree_l = nz - 1;

        let mut result = Vector3::zeros();

        for kk in 0..nz {
            let bw = bernstein_basis(degree_l, kk, param_w);
            for jj in 0..ny {
                let bv = bernstein_basis(degree_m, jj, param_v);
                for ii in 0..nx {
                    let bu = bernstein_basis(degree_n, ii, param_u);

                    let idx = self.control_index(ii, jj, kk);
                    let cp = &self.control_points[idx];
                    let weight = bu * bv * bw;

                    result += cp.coords * weight;
                }
            }
        }

        Point3::from(result)
    }
}

/// Computes the Bernstein basis polynomial B_{i,n}(t).
///
/// # Arguments
///
/// * `n` - The degree of the polynomial
/// * `i` - The index (0 to n)
/// * `t` - The parameter (0 to 1)
#[must_use]
#[allow(clippy::cast_possible_wrap)]
pub fn bernstein_basis(n: usize, i: usize, t: f64) -> f64 {
    #[allow(clippy::cast_precision_loss)]
    let coeff = binomial(n, i) as f64;
    #[allow(clippy::cast_possible_truncation)]
    let ti = t.powi(i as i32);
    #[allow(clippy::cast_possible_truncation)]
    let one_minus_t = (1.0 - t).powi((n - i) as i32);
    coeff * ti * one_minus_t
}

/// Computes the binomial coefficient C(n, k).
#[must_use]
pub const fn binomial(n: usize, k: usize) -> usize {
    if k > n {
        return 0;
    }
    if k == 0 || k == n {
        return 1;
    }

    // Use the smaller k for efficiency
    let k = if k < n - k { k } else { n - k };

    let mut result: usize = 1;
    let mut idx = 0;
    while idx < k {
        result = result * (n - idx) / (idx + 1);
        idx += 1;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_binomial_coefficients() {
        assert_eq!(binomial(0, 0), 1);
        assert_eq!(binomial(1, 0), 1);
        assert_eq!(binomial(1, 1), 1);
        assert_eq!(binomial(4, 0), 1);
        assert_eq!(binomial(4, 1), 4);
        assert_eq!(binomial(4, 2), 6);
        assert_eq!(binomial(4, 3), 4);
        assert_eq!(binomial(4, 4), 1);
        assert_eq!(binomial(5, 2), 10);
        assert_eq!(binomial(10, 5), 252);
    }

    #[test]
    fn test_binomial_out_of_range() {
        assert_eq!(binomial(3, 5), 0);
    }

    #[test]
    fn test_bernstein_at_endpoints() {
        // At t=0, B_{0,n}(0) = 1 and B_{i,n}(0) = 0 for i > 0
        assert_relative_eq!(bernstein_basis(3, 0, 0.0), 1.0, epsilon = 1e-10);
        assert_relative_eq!(bernstein_basis(3, 1, 0.0), 0.0, epsilon = 1e-10);
        assert_relative_eq!(bernstein_basis(3, 2, 0.0), 0.0, epsilon = 1e-10);
        assert_relative_eq!(bernstein_basis(3, 3, 0.0), 0.0, epsilon = 1e-10);

        // At t=1, B_{n,n}(1) = 1 and B_{i,n}(1) = 0 for i < n
        assert_relative_eq!(bernstein_basis(3, 0, 1.0), 0.0, epsilon = 1e-10);
        assert_relative_eq!(bernstein_basis(3, 1, 1.0), 0.0, epsilon = 1e-10);
        assert_relative_eq!(bernstein_basis(3, 2, 1.0), 0.0, epsilon = 1e-10);
        assert_relative_eq!(bernstein_basis(3, 3, 1.0), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_bernstein_partition_of_unity() {
        // Sum of all Bernstein polynomials equals 1
        for n in 1..=5 {
            for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let sum: f64 = (0..=n).map(|i| bernstein_basis(n, i, t)).sum();
                assert_relative_eq!(sum, 1.0, epsilon = 1e-10);
            }
        }
    }

    #[test]
    fn test_ffd_config_validation() {
        assert!(FfdConfig::new(2, 2, 2).is_ok());
        assert!(FfdConfig::new(4, 4, 4).is_ok());
        assert!(FfdConfig::new(1, 2, 2).is_err());
        assert!(FfdConfig::new(2, 1, 2).is_err());
        assert!(FfdConfig::new(2, 2, 1).is_err());
    }

    #[test]
    fn test_ffd_lattice_identity() {
        let bounds = Aabb {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(1.0, 1.0, 1.0),
        };
        let config = FfdConfig::new(4, 4, 4).unwrap();
        let lattice = FfdLattice::new(&bounds, config);

        // Without any constraints, points should stay (approximately) in place
        // The lattice bounds have padding, so there's some small offset
        let test_points = [
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(0.25, 0.25, 0.25),
            Point3::new(0.75, 0.75, 0.75),
        ];

        for p in &test_points {
            let result = lattice.transform(p);
            // Should be close to original (within padding tolerance)
            assert!(
                (result - p).norm() < 0.1,
                "Point {:?} moved too much to {:?}",
                p,
                result
            );
        }
    }

    #[test]
    fn test_ffd_lattice_with_displacement() {
        let bounds = Aabb {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(2.0, 2.0, 2.0),
        };
        let config = FfdConfig::new(3, 3, 3).unwrap();
        let mut lattice = FfdLattice::new(&bounds, config);

        // Apply a constraint at the center
        let constraint =
            Constraint::displacement(Point3::new(1.0, 1.0, 1.0), Vector3::new(0.0, 0.0, 0.5));
        lattice.apply_constraints(&[constraint]);

        // The center should have some displacement
        let center = Point3::new(1.0, 1.0, 1.0);
        let result = lattice.transform(&center);
        assert!(result.z > center.z, "Expected positive Z displacement");
    }

    #[test]
    fn test_world_to_lattice_coords() {
        let bounds = Aabb {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(10.0, 10.0, 10.0),
        };
        let config = FfdConfig::new(4, 4, 4).unwrap().with_padding(0.0);
        let lattice = FfdLattice::new(&bounds, config);

        // Test corner points
        let (u, v, w) = lattice.world_to_lattice(&Point3::new(0.0, 0.0, 0.0));
        assert_relative_eq!(u, 0.0, epsilon = 1e-10);
        assert_relative_eq!(v, 0.0, epsilon = 1e-10);
        assert_relative_eq!(w, 0.0, epsilon = 1e-10);

        let (u, v, w) = lattice.world_to_lattice(&Point3::new(10.0, 10.0, 10.0));
        assert_relative_eq!(u, 1.0, epsilon = 1e-10);
        assert_relative_eq!(v, 1.0, epsilon = 1e-10);
        assert_relative_eq!(w, 1.0, epsilon = 1e-10);

        // Test center
        let (u, v, w) = lattice.world_to_lattice(&Point3::new(5.0, 5.0, 5.0));
        assert_relative_eq!(u, 0.5, epsilon = 1e-10);
        assert_relative_eq!(v, 0.5, epsilon = 1e-10);
        assert_relative_eq!(w, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_control_point_count() {
        let config = FfdConfig::new(4, 5, 6).unwrap();
        assert_eq!(config.control_point_count(), 4 * 5 * 6);
    }
}
