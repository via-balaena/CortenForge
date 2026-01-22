//! Printer configuration and technology types.
//!
//! Provides configuration for different 3D printing technologies
//! (FDM, SLA, SLS) with their specific constraints and capabilities.

/// 3D printing technology.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PrintTechnology {
    /// Fused Deposition Modeling (filament).
    Fdm,
    /// Stereolithography (resin).
    Sla,
    /// Selective Laser Sintering (powder).
    Sls,
    /// Multi Jet Fusion (powder, HP).
    Mjf,
    /// Other/custom technology.
    Other,
}

impl PrintTechnology {
    /// Get a human-readable name for the technology.
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Fdm => "FDM",
            Self::Sla => "SLA",
            Self::Sls => "SLS",
            Self::Mjf => "MJF",
            Self::Other => "Other",
        }
    }

    /// Check if this technology typically requires support structures.
    #[must_use]
    pub fn requires_supports(&self) -> bool {
        match self {
            Self::Fdm | Self::Sla => true,
            Self::Sls | Self::Mjf => false,
            Self::Other => true,
        }
    }
}

/// Configuration for a specific 3D printer.
///
/// Contains parameters that define the capabilities and constraints
/// of a 3D printer for validation purposes.
///
/// # Example
///
/// ```
/// use mesh_printability::{PrinterConfig, PrintTechnology};
///
/// let config = PrinterConfig::fdm_default();
/// assert_eq!(config.technology, PrintTechnology::Fdm);
/// assert!(config.min_wall_thickness > 0.0);
/// ```
#[derive(Debug, Clone)]
pub struct PrinterConfig {
    /// Printer technology (FDM, SLA, SLS, etc.).
    pub technology: PrintTechnology,

    /// Minimum wall thickness in mm.
    pub min_wall_thickness: f64,

    /// Nozzle diameter in mm (for FDM).
    pub nozzle_diameter: f64,

    /// Layer height in mm.
    pub layer_height: f64,

    /// Maximum overhang angle in degrees (0 = vertical, 90 = horizontal).
    pub max_overhang_angle: f64,

    /// Minimum feature size that can be printed in mm.
    pub min_feature_size: f64,

    /// Maximum bridge span in mm.
    pub max_bridge_span: f64,

    /// Build volume (X, Y, Z) in mm.
    pub build_volume: (f64, f64, f64),
}

impl Default for PrinterConfig {
    fn default() -> Self {
        Self::fdm_default()
    }
}

impl PrinterConfig {
    /// Default configuration for FDM printers.
    ///
    /// Based on typical consumer FDM printers like Prusa or Ender.
    #[must_use]
    pub fn fdm_default() -> Self {
        Self {
            technology: PrintTechnology::Fdm,
            min_wall_thickness: 1.0,
            nozzle_diameter: 0.4,
            layer_height: 0.2,
            max_overhang_angle: 45.0,
            min_feature_size: 0.8,
            max_bridge_span: 10.0,
            build_volume: (200.0, 200.0, 200.0),
        }
    }

    /// Default configuration for SLA/resin printers.
    ///
    /// Based on typical consumer resin printers.
    #[must_use]
    pub fn sla_default() -> Self {
        Self {
            technology: PrintTechnology::Sla,
            min_wall_thickness: 0.4,
            nozzle_diameter: 0.0, // N/A for SLA
            layer_height: 0.05,
            max_overhang_angle: 30.0, // More conservative for resin
            min_feature_size: 0.1,
            max_bridge_span: 5.0,
            build_volume: (120.0, 68.0, 155.0),
        }
    }

    /// Default configuration for SLS printers.
    ///
    /// Based on typical industrial SLS machines.
    #[must_use]
    pub fn sls_default() -> Self {
        Self {
            technology: PrintTechnology::Sls,
            min_wall_thickness: 0.7,
            nozzle_diameter: 0.0, // N/A for SLS
            layer_height: 0.1,
            max_overhang_angle: 90.0, // SLS doesn't need supports
            min_feature_size: 0.3,
            max_bridge_span: f64::INFINITY, // No bridges needed
            build_volume: (300.0, 300.0, 300.0),
        }
    }

    /// Default configuration for MJF printers (HP Multi Jet Fusion).
    #[must_use]
    pub fn mjf_default() -> Self {
        Self {
            technology: PrintTechnology::Mjf,
            min_wall_thickness: 0.5,
            nozzle_diameter: 0.0, // N/A for MJF
            layer_height: 0.08,
            max_overhang_angle: 90.0, // MJF doesn't need supports
            min_feature_size: 0.2,
            max_bridge_span: f64::INFINITY,
            build_volume: (380.0, 284.0, 380.0),
        }
    }

    /// Create a custom configuration with a specific build volume.
    #[must_use]
    pub fn with_build_volume(mut self, x: f64, y: f64, z: f64) -> Self {
        self.build_volume = (x, y, z);
        self
    }

    /// Create a custom configuration with a specific wall thickness.
    #[must_use]
    pub fn with_min_wall_thickness(mut self, thickness: f64) -> Self {
        self.min_wall_thickness = thickness;
        self
    }

    /// Create a custom configuration with a specific overhang angle.
    #[must_use]
    pub fn with_max_overhang_angle(mut self, angle: f64) -> Self {
        self.max_overhang_angle = angle;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_technology_as_str() {
        assert_eq!(PrintTechnology::Fdm.as_str(), "FDM");
        assert_eq!(PrintTechnology::Sla.as_str(), "SLA");
        assert_eq!(PrintTechnology::Sls.as_str(), "SLS");
        assert_eq!(PrintTechnology::Mjf.as_str(), "MJF");
    }

    #[test]
    fn test_technology_requires_supports() {
        assert!(PrintTechnology::Fdm.requires_supports());
        assert!(PrintTechnology::Sla.requires_supports());
        assert!(!PrintTechnology::Sls.requires_supports());
        assert!(!PrintTechnology::Mjf.requires_supports());
    }

    #[test]
    fn test_fdm_default() {
        let config = PrinterConfig::fdm_default();
        assert_eq!(config.technology, PrintTechnology::Fdm);
        assert!((config.min_wall_thickness - 1.0).abs() < f64::EPSILON);
        assert!((config.nozzle_diameter - 0.4).abs() < f64::EPSILON);
        assert!((config.max_overhang_angle - 45.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_sla_default() {
        let config = PrinterConfig::sla_default();
        assert_eq!(config.technology, PrintTechnology::Sla);
        assert!(config.min_wall_thickness < 1.0); // Thinner walls allowed
        assert!(config.min_feature_size < 0.5); // Finer details
    }

    #[test]
    fn test_sls_default() {
        let config = PrinterConfig::sls_default();
        assert_eq!(config.technology, PrintTechnology::Sls);
        assert!((config.max_overhang_angle - 90.0).abs() < f64::EPSILON); // No supports needed
    }

    #[test]
    fn test_builder_methods() {
        let config = PrinterConfig::fdm_default()
            .with_build_volume(300.0, 300.0, 400.0)
            .with_min_wall_thickness(0.8)
            .with_max_overhang_angle(50.0);

        assert!((config.build_volume.0 - 300.0).abs() < f64::EPSILON);
        assert!((config.build_volume.2 - 400.0).abs() < f64::EPSILON);
        assert!((config.min_wall_thickness - 0.8).abs() < f64::EPSILON);
        assert!((config.max_overhang_angle - 50.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_mjf_default() {
        let config = PrinterConfig::mjf_default();
        assert_eq!(config.technology, PrintTechnology::Mjf);
        assert!((config.max_overhang_angle - 90.0).abs() < f64::EPSILON);
        assert!(config.max_bridge_span.is_infinite());
    }

    #[test]
    fn test_technology_other() {
        assert_eq!(PrintTechnology::Other.as_str(), "Other");
        // Other technology defaults to requiring supports
        assert!(PrintTechnology::Other.requires_supports());
    }

    #[test]
    fn test_default_is_fdm() {
        let config = PrinterConfig::default();
        assert_eq!(config.technology, PrintTechnology::Fdm);
    }
}
