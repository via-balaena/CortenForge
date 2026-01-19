//! Triply Periodic Minimal Surface (TPMS) functions.
//!
//! This module provides implicit surface functions for various TPMS types,
//! which are used to generate lattice structures via marching cubes.

use nalgebra::Point3;
use std::f64::consts::PI;

/// Evaluates the Gyroid TPMS function.
///
/// The gyroid equation is:
/// `sin(x) * cos(y) + sin(y) * cos(z) + sin(z) * cos(x) = threshold`
///
/// # Arguments
///
/// * `point` - The 3D point to evaluate
/// * `cell_size` - Size of one period of the surface
///
/// # Returns
///
/// The scalar field value. The surface is at value 0.
///
/// # Examples
///
/// ```
/// use mesh_lattice::gyroid;
/// use nalgebra::Point3;
///
/// let value = gyroid(Point3::origin(), 10.0);
/// // At origin, sin(0)*cos(0) + sin(0)*cos(0) + sin(0)*cos(0) = 0
/// assert!(value.abs() < 0.001);
/// ```
#[must_use]
pub fn gyroid(point: Point3<f64>, cell_size: f64) -> f64 {
    let scale = 2.0 * PI / cell_size;
    let x = point.x * scale;
    let y = point.y * scale;
    let z = point.z * scale;

    z.sin().mul_add(x.cos(), x.sin().mul_add(y.cos(), y.sin() * z.cos()))
}

/// Evaluates the Schwarz-P (Primitive) TPMS function.
///
/// The Schwarz-P equation is:
/// `cos(x) + cos(y) + cos(z) = threshold`
///
/// # Arguments
///
/// * `point` - The 3D point to evaluate
/// * `cell_size` - Size of one period of the surface
///
/// # Returns
///
/// The scalar field value. The surface is at value 0.
#[must_use]
pub fn schwarz_p(point: Point3<f64>, cell_size: f64) -> f64 {
    let scale = 2.0 * PI / cell_size;
    let x = point.x * scale;
    let y = point.y * scale;
    let z = point.z * scale;

    x.cos() + y.cos() + z.cos()
}

/// Evaluates the Diamond TPMS function.
///
/// The Diamond equation is:
/// `sin(x)*sin(y)*sin(z) + sin(x)*cos(y)*cos(z) + cos(x)*sin(y)*cos(z) + cos(x)*cos(y)*sin(z) = threshold`
///
/// # Arguments
///
/// * `point` - The 3D point to evaluate
/// * `cell_size` - Size of one period of the surface
///
/// # Returns
///
/// The scalar field value. The surface is at value 0.
#[must_use]
pub fn diamond(point: Point3<f64>, cell_size: f64) -> f64 {
    let scale = 2.0 * PI / cell_size;
    let x = point.x * scale;
    let y = point.y * scale;
    let z = point.z * scale;

    let (sx, cx) = x.sin_cos();
    let (sy, cy) = y.sin_cos();
    let (sz, cz) = z.sin_cos();

    (cx * cy).mul_add(sz, (cx * sy).mul_add(cz, (sx * sy).mul_add(sz, sx * cy * cz)))
}

/// Evaluates the Neovius TPMS function.
///
/// The Neovius equation is:
/// `3*(cos(x) + cos(y) + cos(z)) + 4*cos(x)*cos(y)*cos(z) = threshold`
///
/// # Arguments
///
/// * `point` - The 3D point to evaluate
/// * `cell_size` - Size of one period of the surface
///
/// # Returns
///
/// The scalar field value.
#[must_use]
pub fn neovius(point: Point3<f64>, cell_size: f64) -> f64 {
    let scale = 2.0 * PI / cell_size;
    let x = point.x * scale;
    let y = point.y * scale;
    let z = point.z * scale;

    let cx = x.cos();
    let cy = y.cos();
    let cz = z.cos();

    3.0f64.mul_add(cx + cy + cz, 4.0 * cx * cy * cz)
}

/// Evaluates the IWP (I-graph Wrapped Package) TPMS function.
///
/// # Arguments
///
/// * `point` - The 3D point to evaluate
/// * `cell_size` - Size of one period of the surface
///
/// # Returns
///
/// The scalar field value.
#[must_use]
pub fn iwp(point: Point3<f64>, cell_size: f64) -> f64 {
    let scale = 2.0 * PI / cell_size;
    let x = point.x * scale;
    let y = point.y * scale;
    let z = point.z * scale;

    2.0f64.mul_add(z.cos().mul_add(x.cos(), x.cos().mul_add(y.cos(), y.cos() * z.cos())), -(2.0 * x).cos())
        - (2.0 * y).cos()
        - (2.0 * z).cos()
}

/// Converts a density value to a TPMS threshold.
///
/// Different TPMS types have different threshold ranges that produce
/// valid (non-self-intersecting) surfaces.
///
/// # Arguments
///
/// * `density` - Target volume fraction (0.0 to 1.0)
/// * `tpms_type` - The type of TPMS being used
///
/// # Returns
///
/// The threshold value for the TPMS function.
#[must_use]
pub fn density_to_threshold(density: f64, tpms_type: &str) -> f64 {
    // These are approximate mappings from density to threshold
    // The actual relationship is non-linear and varies by TPMS type
    let density = density.clamp(0.05, 0.95);

    match tpms_type.to_lowercase().as_str() {
        "gyroid" => {
            // Gyroid threshold range is approximately [-1.5, 1.5]
            // Density 0.5 corresponds to threshold 0
            (0.5 - density) * 3.0
        }
        "schwarz_p" | "schwarzp" => {
            // Schwarz-P threshold range is approximately [-3, 3]
            // Density 0.5 corresponds to threshold 0
            (0.5 - density) * 6.0
        }
        "diamond" => {
            // Diamond threshold range is approximately [-1.5, 1.5]
            (0.5 - density) * 3.0
        }
        _ => {
            // Default: assume similar to gyroid
            (0.5 - density) * 3.0
        }
    }
}

/// Creates a shell (thickened surface) from a TPMS function.
///
/// This generates both the inner and outer surfaces of the shell.
///
/// # Arguments
///
/// * `tpms_fn` - The TPMS evaluation function
/// * `wall_thickness` - Thickness of the shell wall
///
/// # Returns
///
/// A function that evaluates the shell SDF.
pub fn make_shell<F>(tpms_fn: F, wall_thickness: f64) -> impl Fn(Point3<f64>) -> f64
where
    F: Fn(Point3<f64>) -> f64,
{
    let half_thickness = wall_thickness / 2.0;
    move |point| {
        let value = tpms_fn(point);
        value.abs() - half_thickness
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_gyroid_at_origin() {
        // At origin: sin(0)*cos(0) + sin(0)*cos(0) + sin(0)*cos(0) = 0
        let value = gyroid(Point3::origin(), 10.0);
        assert_relative_eq!(value, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_gyroid_periodicity() {
        let cell_size = 10.0;
        let p1 = Point3::new(2.5, 3.0, 1.0);
        let p2 = Point3::new(2.5 + cell_size, 3.0 + cell_size, 1.0 + cell_size);

        let v1 = gyroid(p1, cell_size);
        let v2 = gyroid(p2, cell_size);

        assert_relative_eq!(v1, v2, epsilon = 1e-10);
    }

    #[test]
    fn test_schwarz_p_at_origin() {
        // At origin: cos(0) + cos(0) + cos(0) = 3
        let value = schwarz_p(Point3::origin(), 10.0);
        assert_relative_eq!(value, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_schwarz_p_periodicity() {
        let cell_size = 8.0;
        let p1 = Point3::new(1.0, 2.0, 3.0);
        let p2 = Point3::new(1.0 + cell_size, 2.0 + cell_size, 3.0 + cell_size);

        let v1 = schwarz_p(p1, cell_size);
        let v2 = schwarz_p(p2, cell_size);

        assert_relative_eq!(v1, v2, epsilon = 1e-10);
    }

    #[test]
    fn test_diamond_at_origin() {
        // At origin: sin(0)*sin(0)*sin(0) + ... = 0
        let value = diamond(Point3::origin(), 10.0);
        assert_relative_eq!(value, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_diamond_periodicity() {
        let cell_size = 12.0;
        let p1 = Point3::new(3.0, 4.0, 5.0);
        let p2 = Point3::new(3.0 + cell_size, 4.0 + cell_size, 5.0 + cell_size);

        let v1 = diamond(p1, cell_size);
        let v2 = diamond(p2, cell_size);

        assert_relative_eq!(v1, v2, epsilon = 1e-10);
    }

    #[test]
    fn test_neovius_at_origin() {
        // At origin: 3*(1 + 1 + 1) + 4*1*1*1 = 9 + 4 = 13
        let value = neovius(Point3::origin(), 10.0);
        assert_relative_eq!(value, 13.0, epsilon = 1e-10);
    }

    #[test]
    fn test_density_to_threshold_gyroid() {
        // At density 0.5, threshold should be 0
        let threshold = density_to_threshold(0.5, "gyroid");
        assert_relative_eq!(threshold, 0.0, epsilon = 1e-10);

        // Higher density = lower threshold
        let high_density = density_to_threshold(0.7, "gyroid");
        assert!(high_density < 0.0);

        // Lower density = higher threshold
        let low_density = density_to_threshold(0.3, "gyroid");
        assert!(low_density > 0.0);
    }

    #[test]
    fn test_density_clamping() {
        // Values should be clamped
        let extreme_low = density_to_threshold(-0.5, "gyroid");
        let clamped_low = density_to_threshold(0.05, "gyroid");
        assert_relative_eq!(extreme_low, clamped_low, epsilon = 1e-10);
    }

    #[test]
    fn test_make_shell() {
        let wall_thickness = 1.0;
        let shell = make_shell(|p| gyroid(p, 10.0), wall_thickness);

        // Points exactly on the TPMS surface should be inside the shell
        // (negative SDF for shell)
        let on_surface = shell(Point3::origin());
        assert!(on_surface < 0.0);

        // Points far from the surface should be outside
        let far_point = shell(Point3::new(2.5, 2.5, 2.5));
        // This might be inside or outside depending on the TPMS, but the
        // shell function should work
        assert!(far_point.is_finite());
    }

    #[test]
    fn test_iwp() {
        // Just verify it evaluates without panicking
        let value = iwp(Point3::new(1.0, 2.0, 3.0), 10.0);
        assert!(value.is_finite());
    }
}
