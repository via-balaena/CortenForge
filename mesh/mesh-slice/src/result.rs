//! Slice result types.

// Layer counts don't overflow in practice
#![allow(clippy::cast_precision_loss)]

use crate::layer::{Layer, LayerStats};
use crate::params::SliceParams;

/// Result of slicing operation.
#[derive(Debug)]
pub struct SliceResult {
    /// Individual layers from bottom to top.
    pub layers: Vec<Layer>,

    /// Total height of the sliced object in mm.
    pub total_height: f64,

    /// Total number of layers.
    pub layer_count: usize,

    /// Estimated print time in minutes.
    pub estimated_print_time: f64,

    /// Estimated filament usage in mm.
    pub estimated_filament_length: f64,

    /// Estimated filament volume in mm³.
    pub estimated_filament_volume: f64,

    /// Layer with maximum area.
    pub max_area_layer: usize,

    /// Layer with maximum perimeter.
    pub max_perimeter_layer: usize,

    /// Slice parameters used.
    pub params: SliceParams,
}

impl SliceResult {
    /// Create an empty result.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)] // Vec::new() is not const
    pub fn empty(params: SliceParams) -> Self {
        Self {
            layers: Vec::new(),
            total_height: 0.0,
            layer_count: 0,
            estimated_print_time: 0.0,
            estimated_filament_length: 0.0,
            estimated_filament_volume: 0.0,
            max_area_layer: 0,
            max_perimeter_layer: 0,
            params,
        }
    }

    /// Check if the result is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.layers.is_empty()
    }

    /// Calculate layer statistics for all layers.
    #[must_use]
    pub fn stats(&self) -> LayerStats {
        if self.layers.is_empty() {
            return LayerStats::default();
        }

        let mut min_area = f64::INFINITY;
        let mut max_area: f64 = 0.0;
        let mut sum_area: f64 = 0.0;
        let mut min_perimeter = f64::INFINITY;
        let mut max_perimeter: f64 = 0.0;
        let mut sum_perimeter: f64 = 0.0;
        let mut max_islands: usize = 0;

        for layer in &self.layers {
            min_area = min_area.min(layer.area);
            max_area = max_area.max(layer.area);
            sum_area += layer.area;
            min_perimeter = min_perimeter.min(layer.perimeter);
            max_perimeter = max_perimeter.max(layer.perimeter);
            sum_perimeter += layer.perimeter;
            max_islands = max_islands.max(layer.island_count);
        }

        let n = self.layers.len() as f64;
        LayerStats {
            min_area,
            max_area,
            avg_area: sum_area / n,
            min_perimeter,
            max_perimeter,
            avg_perimeter: sum_perimeter / n,
            max_islands,
        }
    }

    /// Get a specific layer by index.
    #[must_use]
    pub fn get_layer(&self, index: usize) -> Option<&Layer> {
        self.layers.get(index)
    }

    /// Get the layer at a specific Z height.
    #[must_use]
    pub fn layer_at_height(&self, z: f64) -> Option<&Layer> {
        self.layers.iter().find(|l| {
            let half_thickness = l.thickness / 2.0;
            z >= l.z_height - half_thickness && z <= l.z_height + half_thickness
        })
    }
}

impl std::fmt::Display for SliceResult {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "SliceResult: {} layers, {:.1}mm height, ~{:.1}min print time",
            self.layer_count, self.total_height, self.estimated_print_time
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_result() {
        let result = SliceResult::empty(SliceParams::default());
        assert!(result.is_empty());
        assert_eq!(result.layer_count, 0);
    }

    #[test]
    fn test_display() {
        let result = SliceResult::empty(SliceParams::default());
        let display = format!("{result}");
        assert!(display.contains("0 layers"));
    }
}
