//! Density mapping for variable-density lattices.

use nalgebra::Point3;
use std::sync::Arc;

/// Density map for variable-density lattice generation.
///
/// Density maps control how the lattice density varies spatially.
/// All densities are in the range [0.0, 1.0].
///
/// # Examples
///
/// ```
/// use mesh_lattice::DensityMap;
/// use nalgebra::Point3;
///
/// // Uniform 30% density
/// let uniform = DensityMap::Uniform(0.3);
/// assert!((uniform.evaluate(Point3::origin()) - 0.3).abs() < 0.001);
///
/// // Gradient from bottom to top
/// let gradient = DensityMap::Gradient {
///     from: Point3::new(0.0, 0.0, 0.0),
///     from_density: 0.1,
///     to: Point3::new(0.0, 0.0, 100.0),
///     to_density: 0.5,
/// };
/// ```
#[derive(Clone)]
pub enum DensityMap {
    /// Uniform density everywhere.
    Uniform(f64),

    /// Linear gradient between two points.
    Gradient {
        /// Start point of the gradient.
        from: Point3<f64>,
        /// Density at the start point.
        from_density: f64,
        /// End point of the gradient.
        to: Point3<f64>,
        /// Density at the end point.
        to_density: f64,
    },

    /// Radial density from a center point.
    Radial {
        /// Center of the radial field.
        center: Point3<f64>,
        /// Inner radius (full inner density).
        inner_radius: f64,
        /// Density at the inner radius.
        inner_density: f64,
        /// Outer radius (full outer density).
        outer_radius: f64,
        /// Density at the outer radius.
        outer_density: f64,
    },

    /// Density based on distance from the mesh surface.
    ///
    /// Requires external distance computation.
    SurfaceDistance {
        /// Density at the surface.
        surface_density: f64,
        /// Density at the core (far from surface).
        core_density: f64,
        /// Depth at which transition completes.
        transition_depth: f64,
    },

    /// Density based on stress field.
    ///
    /// Higher stress areas get higher density.
    StressField {
        /// Function that returns stress magnitude at a point.
        stress_lookup: Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync>,
        /// Minimum density (at zero stress).
        min_density: f64,
        /// Maximum density (at max stress).
        max_density: f64,
        /// Exponent for stress-to-density mapping.
        /// 1.0 = linear, >1.0 = emphasize high-stress regions.
        stress_exponent: f64,
    },

    /// Custom density function.
    Function(Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync>),
}

impl std::fmt::Debug for DensityMap {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Uniform(d) => f.debug_tuple("Uniform").field(d).finish(),
            Self::Gradient {
                from,
                from_density,
                to,
                to_density,
            } => f
                .debug_struct("Gradient")
                .field("from", from)
                .field("from_density", from_density)
                .field("to", to)
                .field("to_density", to_density)
                .finish(),
            Self::Radial {
                center,
                inner_radius,
                inner_density,
                outer_radius,
                outer_density,
            } => f
                .debug_struct("Radial")
                .field("center", center)
                .field("inner_radius", inner_radius)
                .field("inner_density", inner_density)
                .field("outer_radius", outer_radius)
                .field("outer_density", outer_density)
                .finish(),
            Self::SurfaceDistance {
                surface_density,
                core_density,
                transition_depth,
            } => f
                .debug_struct("SurfaceDistance")
                .field("surface_density", surface_density)
                .field("core_density", core_density)
                .field("transition_depth", transition_depth)
                .finish(),
            Self::StressField {
                min_density,
                max_density,
                stress_exponent,
                ..
            } => f
                .debug_struct("StressField")
                .field("min_density", min_density)
                .field("max_density", max_density)
                .field("stress_exponent", stress_exponent)
                .finish(),
            Self::Function(_) => f.debug_tuple("Function").field(&"<fn>").finish(),
        }
    }
}

impl DensityMap {
    /// Evaluates the density at a given point.
    ///
    /// Returns a value in [0.0, 1.0].
    ///
    /// # Arguments
    ///
    /// * `point` - The 3D point to evaluate
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::DensityMap;
    /// use nalgebra::Point3;
    ///
    /// let map = DensityMap::Uniform(0.5);
    /// let density = map.evaluate(Point3::new(10.0, 20.0, 30.0));
    /// assert!((density - 0.5).abs() < 0.001);
    /// ```
    #[must_use]
    pub fn evaluate(&self, point: Point3<f64>) -> f64 {
        let raw = match self {
            Self::Uniform(d) => *d,
            Self::Gradient {
                from,
                from_density,
                to,
                to_density,
            } => {
                let direction = to - from;
                let length_sq = direction.norm_squared();
                if length_sq < f64::EPSILON {
                    *from_density
                } else {
                    let t = (point - from).dot(&direction) / length_sq;
                    let t = t.clamp(0.0, 1.0);
                    from_density + t * (to_density - from_density)
                }
            }
            Self::Radial {
                center,
                inner_radius,
                inner_density,
                outer_radius,
                outer_density,
            } => {
                let dist = (point - center).norm();
                if dist <= *inner_radius {
                    *inner_density
                } else if dist >= *outer_radius {
                    *outer_density
                } else {
                    let t = (dist - inner_radius) / (outer_radius - inner_radius);
                    inner_density + t * (outer_density - inner_density)
                }
            }
            Self::SurfaceDistance {
                surface_density,
                core_density,
                transition_depth: _,
            } => {
                // This requires external distance computation.
                // Without mesh context, we return the average.
                // In practice, this is used with `evaluate_with_distance`.
                f64::midpoint(*surface_density, *core_density)
            }
            Self::StressField {
                stress_lookup,
                min_density,
                max_density,
                stress_exponent,
            } => {
                let stress = stress_lookup(point).clamp(0.0, 1.0);
                let mapped = stress.powf(*stress_exponent);
                min_density + mapped * (max_density - min_density)
            }
            Self::Function(f) => f(point),
        };
        raw.clamp(0.0, 1.0)
    }

    /// Evaluates surface-distance based density.
    ///
    /// This is used when you have precomputed the distance to the mesh surface.
    ///
    /// # Arguments
    ///
    /// * `distance` - Distance from the mesh surface (positive = inside)
    ///
    /// # Returns
    ///
    /// Returns `None` if this is not a `SurfaceDistance` map.
    #[must_use]
    pub fn evaluate_with_distance(&self, distance: f64) -> Option<f64> {
        match self {
            Self::SurfaceDistance {
                surface_density,
                core_density,
                transition_depth,
            } => {
                let t = (distance / transition_depth).clamp(0.0, 1.0);
                Some(surface_density + t * (core_density - surface_density))
            }
            _ => None,
        }
    }

    /// Creates a stress-based density map with linear mapping.
    ///
    /// # Arguments
    ///
    /// * `stress_fn` - Function returning normalized stress (0.0-1.0) at a point
    /// * `min_density` - Density at zero stress
    /// * `max_density` - Density at maximum stress
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::DensityMap;
    /// use nalgebra::Point3;
    ///
    /// let map = DensityMap::from_stress_field(
    ///     |p| (p.z / 100.0).clamp(0.0, 1.0), // Higher stress at top
    ///     0.1,  // 10% minimum
    ///     0.5,  // 50% maximum
    /// );
    /// ```
    pub fn from_stress_field<F>(stress_fn: F, min_density: f64, max_density: f64) -> Self
    where
        F: Fn(Point3<f64>) -> f64 + Send + Sync + 'static,
    {
        Self::StressField {
            stress_lookup: Arc::new(stress_fn),
            min_density,
            max_density,
            stress_exponent: 1.0,
        }
    }

    /// Creates a stress-based density map with custom exponent.
    ///
    /// # Arguments
    ///
    /// * `stress_fn` - Function returning normalized stress (0.0-1.0) at a point
    /// * `min_density` - Density at zero stress
    /// * `max_density` - Density at maximum stress
    /// * `exponent` - Mapping exponent (1.0 = linear, >1.0 = emphasize high stress)
    pub fn from_stress_field_with_exponent<F>(
        stress_fn: F,
        min_density: f64,
        max_density: f64,
        exponent: f64,
    ) -> Self
    where
        F: Fn(Point3<f64>) -> f64 + Send + Sync + 'static,
    {
        Self::StressField {
            stress_lookup: Arc::new(stress_fn),
            min_density,
            max_density,
            stress_exponent: exponent,
        }
    }

    /// Creates a custom density function.
    ///
    /// # Arguments
    ///
    /// * `f` - Function returning density (0.0-1.0) at any point
    ///
    /// # Examples
    ///
    /// ```
    /// use mesh_lattice::DensityMap;
    /// use nalgebra::Point3;
    ///
    /// let map = DensityMap::from_function(|p| {
    ///     // Sinusoidal density pattern
    ///     0.3 + 0.2 * (p.x / 10.0).sin()
    /// });
    /// ```
    pub fn from_function<F>(f: F) -> Self
    where
        F: Fn(Point3<f64>) -> f64 + Send + Sync + 'static,
    {
        Self::Function(Arc::new(f))
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::suboptimal_flops
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_uniform_density() {
        let map = DensityMap::Uniform(0.4);
        assert_relative_eq!(map.evaluate(Point3::origin()), 0.4, epsilon = 1e-10);
        assert_relative_eq!(
            map.evaluate(Point3::new(100.0, 200.0, 300.0)),
            0.4,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_gradient_density() {
        let map = DensityMap::Gradient {
            from: Point3::new(0.0, 0.0, 0.0),
            from_density: 0.1,
            to: Point3::new(0.0, 0.0, 100.0),
            to_density: 0.9,
        };

        // At start
        assert_relative_eq!(
            map.evaluate(Point3::new(0.0, 0.0, 0.0)),
            0.1,
            epsilon = 1e-10
        );
        // At end
        assert_relative_eq!(
            map.evaluate(Point3::new(0.0, 0.0, 100.0)),
            0.9,
            epsilon = 1e-10
        );
        // At middle
        assert_relative_eq!(
            map.evaluate(Point3::new(0.0, 0.0, 50.0)),
            0.5,
            epsilon = 1e-10
        );
        // Before start (clamped)
        assert_relative_eq!(
            map.evaluate(Point3::new(0.0, 0.0, -50.0)),
            0.1,
            epsilon = 1e-10
        );
        // After end (clamped)
        assert_relative_eq!(
            map.evaluate(Point3::new(0.0, 0.0, 150.0)),
            0.9,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_radial_density() {
        let map = DensityMap::Radial {
            center: Point3::origin(),
            inner_radius: 10.0,
            inner_density: 0.8,
            outer_radius: 50.0,
            outer_density: 0.2,
        };

        // At center
        assert_relative_eq!(map.evaluate(Point3::origin()), 0.8, epsilon = 1e-10);
        // Inside inner radius
        assert_relative_eq!(
            map.evaluate(Point3::new(5.0, 0.0, 0.0)),
            0.8,
            epsilon = 1e-10
        );
        // At inner radius
        assert_relative_eq!(
            map.evaluate(Point3::new(10.0, 0.0, 0.0)),
            0.8,
            epsilon = 1e-10
        );
        // At outer radius
        assert_relative_eq!(
            map.evaluate(Point3::new(50.0, 0.0, 0.0)),
            0.2,
            epsilon = 1e-10
        );
        // Beyond outer radius
        assert_relative_eq!(
            map.evaluate(Point3::new(100.0, 0.0, 0.0)),
            0.2,
            epsilon = 1e-10
        );
        // At midpoint
        assert_relative_eq!(
            map.evaluate(Point3::new(30.0, 0.0, 0.0)),
            0.5,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_stress_field_density() {
        let map = DensityMap::from_stress_field(
            |p| (p.z / 100.0).clamp(0.0, 1.0),
            0.1, // min at z=0
            0.9, // max at z=100
        );

        assert_relative_eq!(
            map.evaluate(Point3::new(0.0, 0.0, 0.0)),
            0.1,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            map.evaluate(Point3::new(0.0, 0.0, 50.0)),
            0.5,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            map.evaluate(Point3::new(0.0, 0.0, 100.0)),
            0.9,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_stress_field_with_exponent() {
        let map = DensityMap::from_stress_field_with_exponent(
            |p| (p.z / 100.0).clamp(0.0, 1.0),
            0.0,
            1.0,
            2.0, // Quadratic
        );

        // At z=50, stress=0.5, mapped = 0.5^2 = 0.25
        assert_relative_eq!(
            map.evaluate(Point3::new(0.0, 0.0, 50.0)),
            0.25,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_custom_function() {
        let map = DensityMap::from_function(|p| 0.5 + 0.1 * p.x);

        assert_relative_eq!(
            map.evaluate(Point3::new(0.0, 0.0, 0.0)),
            0.5,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            map.evaluate(Point3::new(2.0, 0.0, 0.0)),
            0.7,
            epsilon = 1e-10
        );
    }

    #[test]
    fn test_density_clamping() {
        // Custom function that returns out-of-range values
        let map = DensityMap::from_function(|_| 1.5);
        assert_relative_eq!(map.evaluate(Point3::origin()), 1.0, epsilon = 1e-10);

        let map = DensityMap::from_function(|_| -0.5);
        assert_relative_eq!(map.evaluate(Point3::origin()), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_surface_distance_with_distance() {
        let map = DensityMap::SurfaceDistance {
            surface_density: 0.8,
            core_density: 0.2,
            transition_depth: 10.0,
        };

        // Helper to compare Option<f64> with tolerance
        let approx_eq = |opt: Option<f64>, expected: f64| {
            let value = opt.expect("should have value");
            (value - expected).abs() < 1e-10
        };

        // At surface
        assert!(approx_eq(map.evaluate_with_distance(0.0), 0.8));
        // At transition depth
        assert!(approx_eq(map.evaluate_with_distance(10.0), 0.2));
        // At half depth
        assert!(approx_eq(map.evaluate_with_distance(5.0), 0.5));
        // Beyond depth (clamped)
        assert!(approx_eq(map.evaluate_with_distance(20.0), 0.2));
    }

    #[test]
    fn test_debug_impl() {
        let map = DensityMap::Uniform(0.5);
        let debug = format!("{:?}", map);
        assert!(debug.contains("Uniform"));

        let map = DensityMap::from_function(|_| 0.5);
        let debug = format!("{:?}", map);
        assert!(debug.contains("Function"));
    }
}
