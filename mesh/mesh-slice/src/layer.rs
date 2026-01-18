//! Layer types for sliced meshes.

use mesh_types::Point3;

/// A single layer/slice of the mesh.
#[derive(Debug, Clone)]
pub struct Layer {
    /// Layer index (0 = first layer).
    pub index: usize,

    /// Z height of this layer in mm.
    pub z_height: f64,

    /// Layer thickness in mm.
    pub thickness: f64,

    /// Cross-section contours for this layer.
    pub contours: Vec<Contour>,

    /// Total area of all contours in mm².
    pub area: f64,

    /// Total perimeter length in mm.
    pub perimeter: f64,

    /// Estimated print time for this layer in seconds.
    pub print_time: f64,

    /// Estimated filament length for this layer in mm.
    pub filament_length: f64,

    /// Number of separate islands (disconnected regions).
    pub island_count: usize,

    /// Bounding box of this layer (2D).
    pub bounds: LayerBounds,
}

impl Layer {
    /// Check if the layer is empty (no contours).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.contours.is_empty()
    }

    /// Get the number of contours.
    #[must_use]
    pub fn contour_count(&self) -> usize {
        self.contours.len()
    }
}

/// A contour (closed loop) in a layer.
#[derive(Debug, Clone)]
pub struct Contour {
    /// Points defining the contour (closed loop).
    pub points: Vec<Point3<f64>>,

    /// Area enclosed by this contour.
    pub area: f64,

    /// Perimeter of this contour.
    pub perimeter: f64,

    /// Whether this is an outer contour (vs hole).
    pub is_outer: bool,

    /// Centroid of the contour.
    pub centroid: Point3<f64>,
}

impl Contour {
    /// Check if the contour is a hole (inner contour).
    #[must_use]
    pub const fn is_hole(&self) -> bool {
        !self.is_outer
    }

    /// Get the number of points in the contour.
    #[must_use]
    pub fn point_count(&self) -> usize {
        self.points.len()
    }
}

/// 2D bounding box for a layer.
#[derive(Debug, Clone, Copy, Default)]
pub struct LayerBounds {
    /// Minimum X coordinate.
    pub min_x: f64,
    /// Maximum X coordinate.
    pub max_x: f64,
    /// Minimum Y coordinate.
    pub min_y: f64,
    /// Maximum Y coordinate.
    pub max_y: f64,
}

impl LayerBounds {
    /// Width of the bounding box.
    #[must_use]
    pub fn width(&self) -> f64 {
        self.max_x - self.min_x
    }

    /// Height of the bounding box.
    #[must_use]
    pub fn height(&self) -> f64 {
        self.max_y - self.min_y
    }

    /// Center point of the bounding box.
    #[must_use]
    pub const fn center(&self) -> (f64, f64) {
        (
            f64::midpoint(self.min_x, self.max_x),
            f64::midpoint(self.min_y, self.max_y),
        )
    }

    /// Check if a point is inside the bounds.
    #[must_use]
    pub fn contains(&self, x: f64, y: f64) -> bool {
        x >= self.min_x && x <= self.max_x && y >= self.min_y && y <= self.max_y
    }
}

/// Layer statistics summary.
#[derive(Debug, Clone, Default)]
pub struct LayerStats {
    /// Minimum area across all layers.
    pub min_area: f64,
    /// Maximum area across all layers.
    pub max_area: f64,
    /// Average area.
    pub avg_area: f64,
    /// Minimum perimeter.
    pub min_perimeter: f64,
    /// Maximum perimeter.
    pub max_perimeter: f64,
    /// Average perimeter.
    pub avg_perimeter: f64,
    /// Maximum island count.
    pub max_islands: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_layer_bounds() {
        let bounds = LayerBounds {
            min_x: 0.0,
            max_x: 10.0,
            min_y: 0.0,
            max_y: 5.0,
        };

        assert!((bounds.width() - 10.0).abs() < f64::EPSILON);
        assert!((bounds.height() - 5.0).abs() < f64::EPSILON);

        let (cx, cy) = bounds.center();
        assert!((cx - 5.0).abs() < f64::EPSILON);
        assert!((cy - 2.5).abs() < f64::EPSILON);

        assert!(bounds.contains(5.0, 2.5));
        assert!(!bounds.contains(11.0, 2.5));
    }

    #[test]
    fn test_contour_is_hole() {
        let contour = Contour {
            points: vec![],
            area: 1.0,
            perimeter: 4.0,
            is_outer: false,
            centroid: Point3::origin(),
        };

        assert!(contour.is_hole());
        assert!(!contour.is_outer);
    }
}
