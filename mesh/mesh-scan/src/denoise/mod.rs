//! Mesh denoising algorithms.
//!
//! This module provides algorithms for smoothing noisy mesh surfaces while
//! preserving important geometric features:
//!
//! - **Laplacian** - Simple, fast smoothing (may cause shrinkage)
//! - **Taubin** - Shrink-free smoothing using alternating passes
//! - **Bilateral** - Feature-preserving smoothing using normal similarity
//!
//! # Quick Start
//!
//! ```
//! use mesh_scan::denoise::{denoise_mesh, DenoiseParams, DenoiseMethod};
//! use mesh_types::{IndexedMesh, Vertex};
//!
//! let mut mesh = IndexedMesh::new();
//! // ... create or load noisy mesh ...
//! mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
//! mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.1)); // Slightly noisy
//! mesh.faces.push([0, 1, 2]);
//!
//! let params = DenoiseParams::for_scans();
//! let result = denoise_mesh(&mesh, &params).unwrap();
//!
//! println!("{}", result);
//! ```
//!
//! # Algorithm Selection
//!
//! | Algorithm | Preserves Volume | Preserves Features | Speed |
//! |-----------|------------------|-------------------|-------|
//! | Laplacian | No (shrinks) | No | Fast |
//! | Taubin | Yes | No | Medium |
//! | Bilateral | Yes | Yes | Slower |

pub mod bilateral;
pub mod laplacian;
pub mod taubin;

pub use bilateral::{BilateralParams, BilateralResult, filter_bilateral};
pub use laplacian::{LaplacianResult, smooth_laplacian, smooth_laplacian_iterations};
pub use taubin::{TaubinParams, TaubinResult, smooth_taubin, smooth_taubin_iterations};

use mesh_types::IndexedMesh;

use crate::error::{ScanError, ScanResult};

/// Denoising method to use.
#[derive(Debug, Clone)]
pub enum DenoiseMethod {
    /// Laplacian smoothing with the given lambda factor.
    Laplacian {
        /// Smoothing strength (0.0-1.0). Default: 0.5.
        lambda: f64,
    },

    /// Taubin shrink-free smoothing.
    Taubin {
        /// Smoothing factor. Default: 0.5.
        lambda: f64,
        /// Inflation factor. Default: -0.53.
        mu: f64,
    },

    /// Bilateral feature-preserving filtering.
    Bilateral {
        /// Spatial sigma. Default: 1.0.
        sigma_spatial: f64,
        /// Normal sigma. Default: 0.5.
        sigma_normal: f64,
    },
}

impl Default for DenoiseMethod {
    fn default() -> Self {
        Self::Taubin {
            lambda: 0.5,
            mu: -0.53,
        }
    }
}

/// Parameters for mesh denoising.
#[derive(Debug, Clone)]
pub struct DenoiseParams {
    /// The denoising method to use.
    pub method: DenoiseMethod,

    /// Number of iterations. Default: 5.
    pub iterations: u32,

    /// Whether to preserve boundary vertices. Default: true.
    pub preserve_boundaries: bool,
}

impl Default for DenoiseParams {
    fn default() -> Self {
        Self {
            method: DenoiseMethod::default(),
            iterations: 5,
            preserve_boundaries: true,
        }
    }
}

impl DenoiseParams {
    /// Creates new parameters with defaults.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Creates parameters using Laplacian smoothing.
    #[must_use]
    pub const fn laplacian(iterations: u32) -> Self {
        Self {
            method: DenoiseMethod::Laplacian { lambda: 0.5 },
            iterations,
            preserve_boundaries: true,
        }
    }

    /// Creates parameters using Taubin smoothing.
    #[must_use]
    pub const fn taubin(iterations: u32) -> Self {
        Self {
            method: DenoiseMethod::Taubin {
                lambda: 0.5,
                mu: -0.53,
            },
            iterations,
            preserve_boundaries: true,
        }
    }

    /// Creates parameters using bilateral filtering.
    #[must_use]
    pub const fn bilateral(iterations: u32) -> Self {
        Self {
            method: DenoiseMethod::Bilateral {
                sigma_spatial: 1.0,
                sigma_normal: 0.5,
            },
            iterations,
            preserve_boundaries: true,
        }
    }

    /// Creates parameters optimized for scan data.
    ///
    /// Uses Taubin smoothing with moderate iterations.
    #[must_use]
    pub const fn for_scans() -> Self {
        Self {
            method: DenoiseMethod::Taubin {
                lambda: 0.5,
                mu: -0.53,
            },
            iterations: 5,
            preserve_boundaries: true,
        }
    }

    /// Creates parameters for aggressive smoothing.
    ///
    /// Uses bilateral filtering with more iterations.
    #[must_use]
    pub const fn aggressive() -> Self {
        Self {
            method: DenoiseMethod::Bilateral {
                sigma_spatial: 2.0,
                sigma_normal: 1.0,
            },
            iterations: 10,
            preserve_boundaries: true,
        }
    }

    /// Creates parameters for gentle smoothing.
    ///
    /// Minimal smoothing that preserves fine details.
    #[must_use]
    pub const fn gentle() -> Self {
        Self {
            method: DenoiseMethod::Bilateral {
                sigma_spatial: 0.5,
                sigma_normal: 0.2,
            },
            iterations: 2,
            preserve_boundaries: true,
        }
    }

    /// Sets the denoising method.
    #[must_use]
    pub const fn with_method(mut self, method: DenoiseMethod) -> Self {
        self.method = method;
        self
    }

    /// Sets the number of iterations.
    #[must_use]
    pub const fn with_iterations(mut self, iterations: u32) -> Self {
        self.iterations = iterations;
        self
    }

    /// Sets whether to preserve boundaries.
    #[must_use]
    pub const fn with_preserve_boundaries(mut self, preserve: bool) -> Self {
        self.preserve_boundaries = preserve;
        self
    }
}

/// Result of mesh denoising.
#[derive(Debug, Clone)]
pub struct DenoiseResult {
    /// The denoised mesh.
    pub mesh: IndexedMesh,

    /// Number of iterations performed.
    pub iterations_performed: u32,

    /// Maximum vertex displacement during denoising.
    pub max_displacement: f64,

    /// Average vertex displacement during denoising.
    pub avg_displacement: f64,
}

impl DenoiseResult {
    /// Returns true if significant denoising occurred.
    #[must_use]
    pub fn had_significant_change(&self) -> bool {
        self.max_displacement > 1e-10
    }
}

impl std::fmt::Display for DenoiseResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Denoise: {} iterations, max displacement: {:.6}, avg: {:.6}",
            self.iterations_performed, self.max_displacement, self.avg_displacement
        )
    }
}

/// Denoises a mesh using the specified parameters.
///
/// # Arguments
///
/// * `mesh` - The mesh to denoise
/// * `params` - Denoising parameters
///
/// # Returns
///
/// The denoised mesh and statistics.
///
/// # Errors
///
/// Returns an error if the mesh is empty.
///
/// # Example
///
/// ```
/// use mesh_scan::denoise::{denoise_mesh, DenoiseParams};
/// use mesh_types::{IndexedMesh, Vertex};
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
/// mesh.vertices.push(Vertex::from_coords(0.5, 1.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let result = denoise_mesh(&mesh, &DenoiseParams::for_scans()).unwrap();
/// println!("{}", result);
/// ```
pub fn denoise_mesh(mesh: &IndexedMesh, params: &DenoiseParams) -> ScanResult<DenoiseResult> {
    if mesh.vertices.is_empty() {
        return Err(ScanError::EmptyMesh);
    }

    if mesh.faces.is_empty() {
        return Ok(DenoiseResult {
            mesh: mesh.clone(),
            iterations_performed: 0,
            max_displacement: 0.0,
            avg_displacement: 0.0,
        });
    }

    match &params.method {
        DenoiseMethod::Laplacian { lambda } => {
            let result = smooth_laplacian_iterations(
                mesh,
                params.iterations,
                *lambda,
                params.preserve_boundaries,
            );

            Ok(DenoiseResult {
                mesh: result.mesh,
                iterations_performed: result.iterations_performed,
                max_displacement: result.max_displacement,
                avg_displacement: result.total_displacement
                    / f64::from(result.iterations_performed.max(1)),
            })
        }

        DenoiseMethod::Taubin { lambda, mu } => {
            let result = smooth_taubin_iterations(
                mesh,
                params.iterations,
                *lambda,
                *mu,
                params.preserve_boundaries,
            );

            Ok(DenoiseResult {
                mesh: result.mesh,
                iterations_performed: result.iterations_performed,
                max_displacement: result.max_displacement,
                avg_displacement: result.total_displacement
                    / f64::from(result.iterations_performed.max(1)),
            })
        }

        DenoiseMethod::Bilateral {
            sigma_spatial,
            sigma_normal,
        } => {
            let bilateral_params = BilateralParams::new()
                .with_sigma_spatial(*sigma_spatial)
                .with_sigma_normal(*sigma_normal)
                .with_iterations(params.iterations);

            let result = filter_bilateral(mesh, &bilateral_params, params.preserve_boundaries);

            Ok(DenoiseResult {
                mesh: result.mesh,
                iterations_performed: result.iterations_performed,
                max_displacement: result.max_displacement,
                avg_displacement: result.avg_displacement,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_types::Vertex;

    fn make_noisy_mesh() -> IndexedMesh {
        use rand::Rng;

        let mut mesh = IndexedMesh::new();
        let mut rng = rand::thread_rng();

        // Create a simple grid
        for i in 0..5 {
            for j in 0..5 {
                let noise: f64 = rng.gen_range(-0.1..0.1);
                mesh.vertices
                    .push(Vertex::from_coords(f64::from(i), f64::from(j), noise));
            }
        }

        // Create triangles
        for i in 0..4 {
            for j in 0..4 {
                #[allow(clippy::cast_possible_truncation)]
                let idx = (i * 5 + j) as u32;
                mesh.faces.push([idx, idx + 1, idx + 5]);
                mesh.faces.push([idx + 1, idx + 6, idx + 5]);
            }
        }

        mesh
    }

    #[test]
    fn test_denoise_params_default() {
        let params = DenoiseParams::default();
        assert!(matches!(params.method, DenoiseMethod::Taubin { .. }));
        assert_eq!(params.iterations, 5);
        assert!(params.preserve_boundaries);
    }

    #[test]
    fn test_denoise_params_factories() {
        let laplacian = DenoiseParams::laplacian(3);
        assert!(matches!(laplacian.method, DenoiseMethod::Laplacian { .. }));
        assert_eq!(laplacian.iterations, 3);

        let taubin = DenoiseParams::taubin(5);
        assert!(matches!(taubin.method, DenoiseMethod::Taubin { .. }));

        let bilateral = DenoiseParams::bilateral(2);
        assert!(matches!(bilateral.method, DenoiseMethod::Bilateral { .. }));
    }

    #[test]
    fn test_denoise_params_presets() {
        let for_scans = DenoiseParams::for_scans();
        assert!(matches!(for_scans.method, DenoiseMethod::Taubin { .. }));

        let aggressive = DenoiseParams::aggressive();
        assert!(matches!(aggressive.method, DenoiseMethod::Bilateral { .. }));
        assert!(aggressive.iterations > 5);

        let gentle = DenoiseParams::gentle();
        assert!(matches!(gentle.method, DenoiseMethod::Bilateral { .. }));
        assert!(gentle.iterations < 5);
    }

    #[test]
    fn test_denoise_params_builder() {
        let params = DenoiseParams::new()
            .with_method(DenoiseMethod::Laplacian { lambda: 0.3 })
            .with_iterations(10)
            .with_preserve_boundaries(false);

        assert!(matches!(params.method, DenoiseMethod::Laplacian { .. }));
        assert_eq!(params.iterations, 10);
        assert!(!params.preserve_boundaries);
    }

    #[test]
    fn test_denoise_mesh_empty() {
        let mesh = IndexedMesh::new();
        let result = denoise_mesh(&mesh, &DenoiseParams::default());
        assert!(matches!(result, Err(ScanError::EmptyMesh)));
    }

    #[test]
    fn test_denoise_mesh_no_faces() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));

        let result = denoise_mesh(&mesh, &DenoiseParams::default()).unwrap();
        assert_eq!(result.iterations_performed, 0);
    }

    #[test]
    fn test_denoise_mesh_laplacian() {
        let mesh = make_noisy_mesh();
        let params = DenoiseParams::laplacian(3);

        let result = denoise_mesh(&mesh, &params).unwrap();

        assert_eq!(result.iterations_performed, 3);
        assert_eq!(result.mesh.vertices.len(), mesh.vertices.len());
    }

    #[test]
    fn test_denoise_mesh_taubin() {
        let mesh = make_noisy_mesh();
        let params = DenoiseParams::taubin(3);

        let result = denoise_mesh(&mesh, &params).unwrap();

        assert_eq!(result.iterations_performed, 3);
        assert_eq!(result.mesh.vertices.len(), mesh.vertices.len());
    }

    #[test]
    fn test_denoise_mesh_bilateral() {
        let mesh = make_noisy_mesh();
        let params = DenoiseParams::bilateral(2);

        let result = denoise_mesh(&mesh, &params).unwrap();

        assert_eq!(result.iterations_performed, 2);
        assert_eq!(result.mesh.vertices.len(), mesh.vertices.len());
    }

    #[test]
    fn test_denoise_result_display() {
        let result = DenoiseResult {
            mesh: IndexedMesh::new(),
            iterations_performed: 5,
            max_displacement: 0.123456,
            avg_displacement: 0.05,
        };

        let display = format!("{result}");
        assert!(display.contains('5'));
        assert!(display.contains("0.123456"));
    }

    #[test]
    fn test_denoise_result_had_significant_change() {
        let no_change = DenoiseResult {
            mesh: IndexedMesh::new(),
            iterations_performed: 5,
            max_displacement: 0.0,
            avg_displacement: 0.0,
        };
        assert!(!no_change.had_significant_change());

        let with_change = DenoiseResult {
            mesh: IndexedMesh::new(),
            iterations_performed: 5,
            max_displacement: 0.1,
            avg_displacement: 0.05,
        };
        assert!(with_change.had_significant_change());
    }
}
