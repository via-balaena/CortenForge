//! Bounding box types for object detection.

use serde::{Deserialize, Serialize};

/// A normalized bounding box with confidence and class.
///
/// Coordinates are normalized to `[0, 1]` range relative to image dimensions.
/// Format is `[x0, y0, x1, y1]` (top-left to bottom-right).
///
/// # Example
///
/// ```
/// use ml_types::BoundingBox;
///
/// let bbox = BoundingBox::new(0.1, 0.2, 0.5, 0.6, 0.95, 0);
///
/// assert!((bbox.width() - 0.4).abs() < 1e-6);
/// assert!((bbox.height() - 0.4).abs() < 1e-6);
/// assert!((bbox.area() - 0.16).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    /// Left edge (x0), normalized to `[0, 1]`.
    pub x0: f32,
    /// Top edge (y0), normalized to `[0, 1]`.
    pub y0: f32,
    /// Right edge (x1), normalized to `[0, 1]`.
    pub x1: f32,
    /// Bottom edge (y1), normalized to `[0, 1]`.
    pub y1: f32,
    /// Confidence score in `[0, 1]`.
    pub confidence: f32,
    /// Class label index.
    pub class_id: u32,
}

impl BoundingBox {
    /// Creates a new bounding box.
    ///
    /// # Arguments
    ///
    /// * `x0`, `y0` - Top-left corner (normalized)
    /// * `x1`, `y1` - Bottom-right corner (normalized)
    /// * `confidence` - Detection confidence `[0, 1]`
    /// * `class_id` - Class label index
    #[must_use]
    pub const fn new(x0: f32, y0: f32, x1: f32, y1: f32, confidence: f32, class_id: u32) -> Self {
        Self {
            x0,
            y0,
            x1,
            y1,
            confidence,
            class_id,
        }
    }

    /// Creates a bounding box from an array `[x0, y0, x1, y1]`.
    #[must_use]
    pub const fn from_array(coords: [f32; 4], confidence: f32, class_id: u32) -> Self {
        Self {
            x0: coords[0],
            y0: coords[1],
            x1: coords[2],
            y1: coords[3],
            confidence,
            class_id,
        }
    }

    /// Returns the box as an array `[x0, y0, x1, y1]`.
    #[must_use]
    pub const fn as_array(&self) -> [f32; 4] {
        [self.x0, self.y0, self.x1, self.y1]
    }

    /// Returns the box width (normalized).
    #[must_use]
    pub fn width(&self) -> f32 {
        (self.x1 - self.x0).max(0.0)
    }

    /// Returns the box height (normalized).
    #[must_use]
    pub fn height(&self) -> f32 {
        (self.y1 - self.y0).max(0.0)
    }

    /// Returns the box area (normalized).
    #[must_use]
    pub fn area(&self) -> f32 {
        self.width() * self.height()
    }

    /// Returns the center point `(cx, cy)`.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn center(&self) -> (f32, f32) {
        (
            f32::midpoint(self.x0, self.x1),
            f32::midpoint(self.y0, self.y1),
        )
    }

    /// Checks if the box coordinates are valid.
    ///
    /// Valid means: in range `[0, 1]` and `x0 <= x1`, `y0 <= y1`.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.x0 >= 0.0
            && self.x0 <= 1.0
            && self.y0 >= 0.0
            && self.y0 <= 1.0
            && self.x1 >= 0.0
            && self.x1 <= 1.0
            && self.y1 >= 0.0
            && self.y1 <= 1.0
            && self.x0 <= self.x1
            && self.y0 <= self.y1
            && !self.x0.is_nan()
            && !self.y0.is_nan()
            && !self.x1.is_nan()
            && !self.y1.is_nan()
    }

    /// Computes the intersection-over-union (`IoU`) with another box.
    ///
    /// Returns a value in `[0, 1]` where 1 means perfect overlap.
    #[must_use]
    #[allow(clippy::similar_names)]
    pub fn iou(&self, other: &Self) -> f32 {
        let inter_x0 = self.x0.max(other.x0);
        let inter_y0 = self.y0.max(other.y0);
        let inter_x1 = self.x1.min(other.x1);
        let inter_y1 = self.y1.min(other.y1);

        let inter_w = (inter_x1 - inter_x0).max(0.0);
        let inter_h = (inter_y1 - inter_y0).max(0.0);
        let inter_area = inter_w * inter_h;

        let union_area = self.area() + other.area() - inter_area;

        if union_area > 0.0 {
            inter_area / union_area
        } else {
            0.0
        }
    }

    /// Checks if this box overlaps with another (`IoU` > 0).
    #[must_use]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.x0 < other.x1 && self.x1 > other.x0 && self.y0 < other.y1 && self.y1 > other.y0
    }

    /// Checks if this box contains a point.
    #[must_use]
    pub fn contains_point(&self, x: f32, y: f32) -> bool {
        x >= self.x0 && x <= self.x1 && y >= self.y0 && y <= self.y1
    }

    /// Converts normalized coordinates to pixel coordinates.
    #[must_use]
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_sign_loss,
        clippy::cast_possible_truncation
    )]
    pub fn to_pixels(&self, width: u32, height: u32) -> (u32, u32, u32, u32) {
        let w = width as f32;
        let h = height as f32;
        (
            (self.x0 * w).round() as u32,
            (self.y0 * h).round() as u32,
            (self.x1 * w).round() as u32,
            (self.y1 * h).round() as u32,
        )
    }

    /// Creates from pixel coordinates.
    #[must_use]
    #[allow(clippy::cast_precision_loss, clippy::too_many_arguments)]
    pub fn from_pixels(
        x0: u32,
        y0: u32,
        x1: u32,
        y1: u32,
        width: u32,
        height: u32,
        confidence: f32,
        class_id: u32,
    ) -> Self {
        let w = width as f32;
        let h = height as f32;
        Self {
            x0: x0 as f32 / w,
            y0: y0 as f32 / h,
            x1: x1 as f32 / w,
            y1: y1 as f32 / h,
            confidence,
            class_id,
        }
    }

    /// Clamps coordinates to valid range `[0, 1]`.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn clamped(&self) -> Self {
        Self {
            x0: self.x0.clamp(0.0, 1.0),
            y0: self.y0.clamp(0.0, 1.0),
            x1: self.x1.clamp(0.0, 1.0),
            y1: self.y1.clamp(0.0, 1.0),
            confidence: self.confidence,
            class_id: self.class_id,
        }
    }
}

impl Default for BoundingBox {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 0)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn bbox_new() {
        let bbox = BoundingBox::new(0.1, 0.2, 0.5, 0.6, 0.9, 1);
        assert!((bbox.x0 - 0.1).abs() < 1e-6);
        assert!((bbox.confidence - 0.9).abs() < 1e-6);
        assert_eq!(bbox.class_id, 1);
    }

    #[test]
    fn bbox_dimensions() {
        let bbox = BoundingBox::new(0.1, 0.2, 0.5, 0.6, 0.9, 0);
        assert!((bbox.width() - 0.4).abs() < 1e-6);
        assert!((bbox.height() - 0.4).abs() < 1e-6);
        assert!((bbox.area() - 0.16).abs() < 1e-6);
    }

    #[test]
    fn bbox_center() {
        let bbox = BoundingBox::new(0.0, 0.0, 1.0, 1.0, 1.0, 0);
        let (cx, cy) = bbox.center();
        assert!((cx - 0.5).abs() < 1e-6);
        assert!((cy - 0.5).abs() < 1e-6);
    }

    #[test]
    fn bbox_validity() {
        let valid = BoundingBox::new(0.1, 0.2, 0.5, 0.6, 0.9, 0);
        assert!(valid.is_valid());

        let invalid_order = BoundingBox::new(0.5, 0.6, 0.1, 0.2, 0.9, 0);
        assert!(!invalid_order.is_valid());

        let out_of_range = BoundingBox::new(-0.1, 0.0, 0.5, 1.5, 0.9, 0);
        assert!(!out_of_range.is_valid());

        let has_nan = BoundingBox::new(f32::NAN, 0.0, 0.5, 0.5, 0.9, 0);
        assert!(!has_nan.is_valid());
    }

    #[test]
    fn bbox_iou() {
        let a = BoundingBox::new(0.0, 0.0, 0.5, 0.5, 1.0, 0);
        let b = BoundingBox::new(0.25, 0.25, 0.75, 0.75, 1.0, 0);

        // Intersection: 0.25 * 0.25 = 0.0625
        // Union: 0.25 + 0.25 - 0.0625 = 0.4375
        // IoU: 0.0625 / 0.4375 ≈ 0.143
        let iou = a.iou(&b);
        assert!(iou > 0.14 && iou < 0.15);

        // Self IoU should be 1.0
        assert!((a.iou(&a) - 1.0).abs() < 1e-6);

        // No overlap
        let c = BoundingBox::new(0.6, 0.6, 0.8, 0.8, 1.0, 0);
        assert!(a.iou(&c).abs() < 1e-6);
    }

    #[test]
    fn bbox_overlaps() {
        let a = BoundingBox::new(0.0, 0.0, 0.5, 0.5, 1.0, 0);
        let b = BoundingBox::new(0.4, 0.4, 0.6, 0.6, 1.0, 0);
        let c = BoundingBox::new(0.6, 0.6, 0.8, 0.8, 1.0, 0);

        assert!(a.overlaps(&b));
        assert!(!a.overlaps(&c));
    }

    #[test]
    fn bbox_contains_point() {
        let bbox = BoundingBox::new(0.1, 0.1, 0.9, 0.9, 1.0, 0);
        assert!(bbox.contains_point(0.5, 0.5));
        assert!(!bbox.contains_point(0.05, 0.5));
    }

    #[test]
    fn bbox_pixel_conversion() {
        let bbox = BoundingBox::new(0.1, 0.2, 0.3, 0.4, 0.9, 0);
        let (px0, py0, px1, py1) = bbox.to_pixels(100, 100);
        assert_eq!(px0, 10);
        assert_eq!(py0, 20);
        assert_eq!(px1, 30);
        assert_eq!(py1, 40);

        let restored = BoundingBox::from_pixels(10, 20, 30, 40, 100, 100, 0.9, 0);
        assert!((restored.x0 - 0.1).abs() < 1e-6);
        assert!((restored.y0 - 0.2).abs() < 1e-6);
    }

    #[test]
    fn bbox_clamped() {
        let out_of_range = BoundingBox::new(-0.1, -0.2, 1.5, 1.3, 0.9, 0);
        let clamped = out_of_range.clamped();
        assert!((clamped.x0 - 0.0).abs() < 1e-6);
        assert!((clamped.y0 - 0.0).abs() < 1e-6);
        assert!((clamped.x1 - 1.0).abs() < 1e-6);
        assert!((clamped.y1 - 1.0).abs() < 1e-6);
    }

    #[test]
    fn bbox_serialization() {
        let bbox = BoundingBox::new(0.1, 0.2, 0.3, 0.4, 0.9, 1);
        let json = serde_json::to_string(&bbox);
        assert!(json.is_ok());

        let parsed: Result<BoundingBox, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }
}
