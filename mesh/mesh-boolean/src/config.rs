//! Configuration and presets for boolean operations.
//!
//! This module provides [`BooleanConfig`] for controlling boolean operation behavior,
//! including tolerance settings, coplanar face handling, and result cleanup options.
//!
//! # Presets
//!
//! Several presets are provided for common use cases:
//!
//! - [`BooleanConfig::default()`] - Balanced settings for general use
//! - [`BooleanConfig::for_scans()`] - Looser tolerances for noisy 3D scan data
//! - [`BooleanConfig::for_cad()`] - Tighter tolerances for precise CAD geometry
//! - [`BooleanConfig::strict()`] - Strictest settings, may fail on imperfect input
//!
//! # Example
//!
//! ```
//! use mesh_boolean::{BooleanConfig, CleanupLevel, CoplanarStrategy};
//!
//! // Use preset for 3D scans
//! let config = BooleanConfig::for_scans();
//!
//! // Or customize settings
//! let config = BooleanConfig::default()
//!     .with_cleanup(CleanupLevel::Full)
//!     .with_coplanar_strategy(CoplanarStrategy::Exclude);
//! ```

/// Level of cleanup to apply to boolean operation results.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CleanupLevel {
    /// No cleanup - return raw boolean output.
    /// Use when you need maximum performance or want to inspect raw results.
    None,

    /// Fast cleanup - weld vertices and remove degenerate triangles.
    /// Good balance of quality and performance for most use cases.
    #[default]
    Fast,

    /// Full cleanup - includes winding repair and non-manifold fixes.
    /// Use when you need guaranteed manifold output for 3D printing.
    Full,
}

/// Strategy for handling coplanar faces during boolean operations.
///
/// When two triangles from different meshes lie in the same plane,
/// special handling is required to avoid ambiguity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CoplanarStrategy {
    /// Include coplanar faces from the first mesh only.
    /// This is the default and produces clean results in most cases.
    #[default]
    Include,

    /// Exclude all coplanar faces from the result.
    /// Useful when coplanar faces cause visual artifacts.
    Exclude,

    /// Keep coplanar faces from both meshes.
    /// May produce non-manifold geometry - use with caution.
    KeepBoth,
}

/// Configuration for boolean operations.
///
/// Controls tolerances, cleanup behavior, and coplanar face handling.
/// Use presets for common scenarios or customize individual settings.
#[derive(Debug, Clone)]
pub struct BooleanConfig {
    /// Tolerance for vertex welding (merging nearby vertices).
    /// Vertices within this distance are considered identical.
    pub vertex_weld_tolerance: f64,

    /// Tolerance for coplanarity detection.
    /// Triangles with vertices within this distance of a plane are considered coplanar.
    pub coplanar_tolerance: f64,

    /// Tolerance for edge-triangle intersection detection.
    pub edge_tolerance: f64,

    /// Tolerance for point-on-plane classification.
    pub classification_tolerance: f64,

    /// Strategy for handling coplanar faces.
    pub coplanar_strategy: CoplanarStrategy,

    /// Level of cleanup to apply to results.
    pub cleanup: CleanupLevel,

    /// Whether to use parallel processing (via rayon).
    pub parallel: bool,

    /// Maximum leaf size for BVH construction.
    /// Smaller values create deeper trees (better for complex intersections).
    /// Larger values create shallower trees (better for simple cases).
    pub bvh_leaf_size: usize,
}

impl Default for BooleanConfig {
    fn default() -> Self {
        Self {
            vertex_weld_tolerance: 1e-6,
            coplanar_tolerance: 1e-6,
            edge_tolerance: 1e-8,
            classification_tolerance: 1e-7,
            coplanar_strategy: CoplanarStrategy::default(),
            cleanup: CleanupLevel::default(),
            parallel: true,
            bvh_leaf_size: 8,
        }
    }
}

impl BooleanConfig {
    /// Create configuration optimized for 3D scan data.
    ///
    /// Uses looser tolerances to handle noise and imperfections
    /// common in scanned meshes.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_boolean::BooleanConfig;
    ///
    /// let config = BooleanConfig::for_scans();
    /// assert!(config.vertex_weld_tolerance > BooleanConfig::default().vertex_weld_tolerance);
    /// ```
    #[must_use]
    pub fn for_scans() -> Self {
        Self {
            vertex_weld_tolerance: 1e-4,
            coplanar_tolerance: 1e-4,
            edge_tolerance: 1e-6,
            classification_tolerance: 1e-5,
            coplanar_strategy: CoplanarStrategy::Include,
            cleanup: CleanupLevel::Full,
            parallel: true,
            bvh_leaf_size: 8,
        }
    }

    /// Create configuration optimized for CAD geometry.
    ///
    /// Uses tighter tolerances appropriate for precise,
    /// mathematically-defined geometry.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_boolean::BooleanConfig;
    ///
    /// let config = BooleanConfig::for_cad();
    /// assert!(config.vertex_weld_tolerance < BooleanConfig::default().vertex_weld_tolerance);
    /// ```
    #[must_use]
    pub fn for_cad() -> Self {
        Self {
            vertex_weld_tolerance: 1e-8,
            coplanar_tolerance: 1e-8,
            edge_tolerance: 1e-10,
            classification_tolerance: 1e-9,
            coplanar_strategy: CoplanarStrategy::Include,
            cleanup: CleanupLevel::Fast,
            parallel: true,
            bvh_leaf_size: 8,
        }
    }

    /// Create strict configuration with tightest tolerances.
    ///
    /// May fail on imperfect input but produces highest quality results
    /// when it succeeds. Use for validated, high-quality input meshes.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_boolean::BooleanConfig;
    ///
    /// let config = BooleanConfig::strict();
    /// assert!(config.vertex_weld_tolerance < BooleanConfig::for_cad().vertex_weld_tolerance);
    /// ```
    #[must_use]
    pub fn strict() -> Self {
        Self {
            vertex_weld_tolerance: 1e-10,
            coplanar_tolerance: 1e-10,
            edge_tolerance: 1e-12,
            classification_tolerance: 1e-11,
            coplanar_strategy: CoplanarStrategy::Include,
            cleanup: CleanupLevel::Fast,
            parallel: true,
            bvh_leaf_size: 4,
        }
    }

    /// Set the cleanup level.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_boolean::{BooleanConfig, CleanupLevel};
    ///
    /// let config = BooleanConfig::default()
    ///     .with_cleanup(CleanupLevel::Full);
    /// ```
    #[must_use]
    pub fn with_cleanup(mut self, level: CleanupLevel) -> Self {
        self.cleanup = level;
        self
    }

    /// Set the coplanar face handling strategy.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_boolean::{BooleanConfig, CoplanarStrategy};
    ///
    /// let config = BooleanConfig::default()
    ///     .with_coplanar_strategy(CoplanarStrategy::Exclude);
    /// ```
    #[must_use]
    pub fn with_coplanar_strategy(mut self, strategy: CoplanarStrategy) -> Self {
        self.coplanar_strategy = strategy;
        self
    }

    /// Enable or disable parallel processing.
    ///
    /// Parallel processing uses rayon for multi-threaded execution.
    /// Disable for deterministic results or when running in single-threaded contexts.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_boolean::BooleanConfig;
    ///
    /// let config = BooleanConfig::default()
    ///     .with_parallel(false);
    /// ```
    #[must_use]
    pub fn with_parallel(mut self, parallel: bool) -> Self {
        self.parallel = parallel;
        self
    }

    /// Set the vertex weld tolerance.
    ///
    /// Vertices within this distance are considered identical and merged.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_boolean::BooleanConfig;
    ///
    /// let config = BooleanConfig::default()
    ///     .with_vertex_weld_tolerance(1e-5);
    /// ```
    #[must_use]
    pub fn with_vertex_weld_tolerance(mut self, tolerance: f64) -> Self {
        self.vertex_weld_tolerance = tolerance.abs();
        self
    }

    /// Set the BVH leaf size.
    ///
    /// Smaller values create deeper trees with better query performance
    /// for complex intersections, but slower construction.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_boolean::BooleanConfig;
    ///
    /// let config = BooleanConfig::default()
    ///     .with_bvh_leaf_size(4);
    /// ```
    #[must_use]
    pub fn with_bvh_leaf_size(mut self, size: usize) -> Self {
        self.bvh_leaf_size = size.max(1);
        self
    }
}

/// Boolean operation type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOp {
    /// Union: A ∪ B
    ///
    /// Combines both meshes, keeping geometry that is outside
    /// the other mesh. Result contains all of A that is outside B,
    /// plus all of B that is outside A.
    Union,

    /// Difference: A - B
    ///
    /// Subtracts mesh B from mesh A. Result contains all of A
    /// that is outside B, plus all of B that is inside A (inverted).
    Difference,

    /// Intersection: A ∩ B
    ///
    /// Keeps only the overlapping region. Result contains all of A
    /// that is inside B, plus all of B that is inside A.
    Intersection,
}

impl std::fmt::Display for BooleanOp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Union => write!(f, "union (A ∪ B)"),
            Self::Difference => write!(f, "difference (A - B)"),
            Self::Intersection => write!(f, "intersection (A ∩ B)"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = BooleanConfig::default();
        assert_eq!(config.cleanup, CleanupLevel::Fast);
        assert_eq!(config.coplanar_strategy, CoplanarStrategy::Include);
        assert!(config.parallel);
    }

    #[test]
    fn test_for_scans_looser_tolerances() {
        let default = BooleanConfig::default();
        let scans = BooleanConfig::for_scans();

        assert!(scans.vertex_weld_tolerance > default.vertex_weld_tolerance);
        assert!(scans.coplanar_tolerance > default.coplanar_tolerance);
    }

    #[test]
    fn test_for_cad_tighter_tolerances() {
        let default = BooleanConfig::default();
        let cad = BooleanConfig::for_cad();

        assert!(cad.vertex_weld_tolerance < default.vertex_weld_tolerance);
        assert!(cad.coplanar_tolerance < default.coplanar_tolerance);
    }

    #[test]
    fn test_strict_tightest_tolerances() {
        let cad = BooleanConfig::for_cad();
        let strict = BooleanConfig::strict();

        assert!(strict.vertex_weld_tolerance < cad.vertex_weld_tolerance);
        assert!(strict.coplanar_tolerance < cad.coplanar_tolerance);
    }

    #[test]
    fn test_builder_methods() {
        let config = BooleanConfig::default()
            .with_cleanup(CleanupLevel::Full)
            .with_coplanar_strategy(CoplanarStrategy::Exclude)
            .with_parallel(false)
            .with_vertex_weld_tolerance(1e-5)
            .with_bvh_leaf_size(16);

        assert_eq!(config.cleanup, CleanupLevel::Full);
        assert_eq!(config.coplanar_strategy, CoplanarStrategy::Exclude);
        assert!(!config.parallel);
        assert!((config.vertex_weld_tolerance - 1e-5).abs() < 1e-12);
        assert_eq!(config.bvh_leaf_size, 16);
    }

    #[test]
    fn test_bvh_leaf_size_minimum() {
        let config = BooleanConfig::default().with_bvh_leaf_size(0);
        assert_eq!(config.bvh_leaf_size, 1);
    }

    #[test]
    fn test_tolerance_abs() {
        let config = BooleanConfig::default().with_vertex_weld_tolerance(-1e-5);
        assert!(config.vertex_weld_tolerance > 0.0);
    }

    #[test]
    fn test_boolean_op_display() {
        assert_eq!(format!("{}", BooleanOp::Union), "union (A ∪ B)");
        assert_eq!(format!("{}", BooleanOp::Difference), "difference (A - B)");
        assert_eq!(
            format!("{}", BooleanOp::Intersection),
            "intersection (A ∩ B)"
        );
    }
}
