//! Slicing parameters and presets.

use mesh_types::Vector3;

/// Parameters for slicing operations.
#[derive(Debug, Clone)]
pub struct SliceParams {
    /// Layer height in mm.
    pub layer_height: f64,

    /// Print direction (typically Z-up).
    pub direction: Vector3<f64>,

    /// First layer height (often thicker for adhesion).
    pub first_layer_height: f64,

    /// Infill density (0.0-1.0).
    pub infill_density: f64,

    /// Number of perimeter shells.
    pub perimeters: usize,

    /// Perimeter width in mm (typically nozzle diameter).
    pub perimeter_width: f64,

    /// Print speed for perimeters in mm/s.
    pub perimeter_speed: f64,

    /// Print speed for infill in mm/s.
    pub infill_speed: f64,

    /// Travel speed in mm/s.
    pub travel_speed: f64,

    /// Extrusion width multiplier.
    pub extrusion_multiplier: f64,
}

impl Default for SliceParams {
    fn default() -> Self {
        Self {
            layer_height: 0.2,
            direction: Vector3::z(),
            first_layer_height: 0.3,
            infill_density: 0.2,
            perimeters: 2,
            perimeter_width: 0.4,
            perimeter_speed: 40.0,
            infill_speed: 60.0,
            travel_speed: 150.0,
            extrusion_multiplier: 1.0,
        }
    }
}

impl SliceParams {
    /// Parameters for high quality printing.
    ///
    /// Uses 0.1mm layers, 3 perimeters, and slower speeds.
    #[must_use]
    pub fn high_quality() -> Self {
        Self {
            layer_height: 0.1,
            first_layer_height: 0.2,
            infill_density: 0.3,
            perimeters: 3,
            perimeter_speed: 30.0,
            infill_speed: 40.0,
            ..Default::default()
        }
    }

    /// Parameters for fast draft printing.
    ///
    /// Uses 0.3mm layers, 2 perimeters, and faster speeds.
    #[must_use]
    pub fn draft() -> Self {
        Self {
            layer_height: 0.3,
            first_layer_height: 0.35,
            infill_density: 0.1,
            perimeters: 2,
            perimeter_speed: 60.0,
            infill_speed: 80.0,
            ..Default::default()
        }
    }

    /// Parameters for SLA/resin printing.
    ///
    /// Uses 0.05mm layers with solid infill.
    #[must_use]
    pub fn for_sla() -> Self {
        Self {
            layer_height: 0.05,
            first_layer_height: 0.05,
            infill_density: 1.0, // SLA typically prints solid
            perimeters: 0,       // Not applicable for SLA
            perimeter_width: 0.0,
            perimeter_speed: 0.0,
            infill_speed: 0.0,
            travel_speed: 0.0,
            ..Default::default()
        }
    }

    /// Set layer height.
    #[must_use]
    pub const fn with_layer_height(mut self, height: f64) -> Self {
        self.layer_height = height;
        self
    }

    /// Set infill density.
    #[must_use]
    pub const fn with_infill(mut self, density: f64) -> Self {
        self.infill_density = density;
        self
    }

    /// Set number of perimeters.
    #[must_use]
    pub const fn with_perimeters(mut self, count: usize) -> Self {
        self.perimeters = count;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_params() {
        let params = SliceParams::default();
        assert!((params.layer_height - 0.2).abs() < 0.001);
        assert_eq!(params.perimeters, 2);
        assert!((params.infill_density - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_high_quality() {
        let params = SliceParams::high_quality();
        assert!(params.layer_height < 0.2);
        assert_eq!(params.perimeters, 3);
    }

    #[test]
    fn test_draft() {
        let params = SliceParams::draft();
        assert!(params.layer_height > 0.2);
        assert!(params.infill_density < 0.2);
    }

    #[test]
    fn test_sla() {
        let params = SliceParams::for_sla();
        assert!((params.infill_density - 1.0).abs() < 0.001);
        assert_eq!(params.perimeters, 0);
    }

    #[test]
    fn test_builder() {
        let params = SliceParams::default()
            .with_layer_height(0.15)
            .with_infill(0.5)
            .with_perimeters(4);

        assert!((params.layer_height - 0.15).abs() < 0.001);
        assert!((params.infill_density - 0.5).abs() < 0.001);
        assert_eq!(params.perimeters, 4);
    }
}
