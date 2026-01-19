//! Detection result types.

use serde::{Deserialize, Serialize};

use crate::BoundingBox;

/// Result of running object detection on a frame.
///
/// Contains all detected objects with their bounding boxes and confidence scores.
///
/// # Example
///
/// ```
/// use ml_types::{DetectionResult, BoundingBox};
///
/// let result = DetectionResult {
///     frame_id: 42,
///     positive: true,
///     confidence: 0.95,
///     boxes: vec![
///         BoundingBox::new(0.1, 0.2, 0.3, 0.4, 0.9, 0),
///         BoundingBox::new(0.5, 0.6, 0.7, 0.8, 0.8, 0),
///     ],
/// };
///
/// assert_eq!(result.detection_count(), 2);
/// assert!(result.has_detections());
/// ```
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct DetectionResult {
    /// Frame ID this result corresponds to.
    pub frame_id: u64,

    /// Whether any detection was found above threshold.
    pub positive: bool,

    /// Overall confidence (typically max of all box scores).
    pub confidence: f32,

    /// Detected bounding boxes with scores.
    pub boxes: Vec<BoundingBox>,
}

impl DetectionResult {
    /// Creates an empty detection result (no detections).
    #[must_use]
    pub const fn empty(frame_id: u64) -> Self {
        Self {
            frame_id,
            positive: false,
            confidence: 0.0,
            boxes: Vec::new(),
        }
    }

    /// Creates a detection result with boxes.
    #[must_use]
    pub fn new(frame_id: u64, boxes: Vec<BoundingBox>) -> Self {
        let positive = !boxes.is_empty();
        let confidence = boxes.iter().map(|b| b.confidence).fold(0.0f32, f32::max);

        Self {
            frame_id,
            positive,
            confidence,
            boxes,
        }
    }

    /// Returns the number of detections.
    #[must_use]
    pub fn detection_count(&self) -> usize {
        self.boxes.len()
    }

    /// Checks if there are any detections.
    #[must_use]
    pub fn has_detections(&self) -> bool {
        !self.boxes.is_empty()
    }

    /// Filters detections by confidence threshold.
    #[must_use]
    pub fn filter_by_confidence(&self, threshold: f32) -> Self {
        let boxes: Vec<_> = self
            .boxes
            .iter()
            .filter(|b| b.confidence >= threshold)
            .copied()
            .collect();

        let positive = !boxes.is_empty();
        let confidence = boxes.iter().map(|b| b.confidence).fold(0.0f32, f32::max);

        Self {
            frame_id: self.frame_id,
            positive,
            confidence,
            boxes,
        }
    }

    /// Filters detections by class ID.
    #[must_use]
    pub fn filter_by_class(&self, class_id: u32) -> Self {
        let boxes: Vec<_> = self
            .boxes
            .iter()
            .filter(|b| b.class_id == class_id)
            .copied()
            .collect();

        Self::new(self.frame_id, boxes)
    }

    /// Returns boxes as arrays `[[x0, y0, x1, y1], ...]` (legacy format).
    #[must_use]
    pub fn boxes_as_arrays(&self) -> Vec<[f32; 4]> {
        self.boxes.iter().map(BoundingBox::as_array).collect()
    }

    /// Returns confidence scores as a vector.
    #[must_use]
    pub fn scores(&self) -> Vec<f32> {
        self.boxes.iter().map(|b| b.confidence).collect()
    }

    /// Applies non-maximum suppression (NMS) to remove overlapping boxes.
    ///
    /// Keeps the highest-confidence box when `IoU` exceeds the threshold.
    #[must_use]
    pub fn nms(&self, iou_threshold: f32) -> Self {
        if self.boxes.is_empty() {
            return self.clone();
        }

        // Sort by confidence (descending)
        let mut boxes = self.boxes.clone();
        boxes.sort_by(|a, b| {
            b.confidence
                .partial_cmp(&a.confidence)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut keep = Vec::new();
        let mut suppressed = vec![false; boxes.len()];

        for i in 0..boxes.len() {
            if suppressed[i] {
                continue;
            }

            keep.push(boxes[i]);

            for j in (i + 1)..boxes.len() {
                if !suppressed[j] && boxes[i].iou(&boxes[j]) > iou_threshold {
                    suppressed[j] = true;
                }
            }
        }

        Self::new(self.frame_id, keep)
    }

    /// Returns the detection with highest confidence.
    #[must_use]
    pub fn best_detection(&self) -> Option<&BoundingBox> {
        self.boxes.iter().max_by(|a, b| {
            a.confidence
                .partial_cmp(&b.confidence)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
    }
}

impl Default for DetectionResult {
    fn default() -> Self {
        Self::empty(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detection_empty() {
        let result = DetectionResult::empty(42);
        assert_eq!(result.frame_id, 42);
        assert!(!result.positive);
        assert!(!result.has_detections());
        assert_eq!(result.detection_count(), 0);
    }

    #[test]
    fn detection_new() {
        let boxes = vec![
            BoundingBox::new(0.1, 0.1, 0.2, 0.2, 0.9, 0),
            BoundingBox::new(0.5, 0.5, 0.6, 0.6, 0.7, 1),
        ];
        let result = DetectionResult::new(1, boxes);

        assert!(result.positive);
        assert!((result.confidence - 0.9).abs() < 1e-6);
        assert_eq!(result.detection_count(), 2);
    }

    #[test]
    fn detection_filter_by_confidence() {
        let boxes = vec![
            BoundingBox::new(0.1, 0.1, 0.2, 0.2, 0.9, 0),
            BoundingBox::new(0.3, 0.3, 0.4, 0.4, 0.5, 0),
            BoundingBox::new(0.5, 0.5, 0.6, 0.6, 0.3, 0),
        ];
        let result = DetectionResult::new(1, boxes);

        let filtered = result.filter_by_confidence(0.6);
        assert_eq!(filtered.detection_count(), 1);
        assert!((filtered.confidence - 0.9).abs() < 1e-6);
    }

    #[test]
    fn detection_filter_by_class() {
        let boxes = vec![
            BoundingBox::new(0.1, 0.1, 0.2, 0.2, 0.9, 0),
            BoundingBox::new(0.3, 0.3, 0.4, 0.4, 0.8, 1),
            BoundingBox::new(0.5, 0.5, 0.6, 0.6, 0.7, 0),
        ];
        let result = DetectionResult::new(1, boxes);

        let class_0 = result.filter_by_class(0);
        assert_eq!(class_0.detection_count(), 2);

        let class_1 = result.filter_by_class(1);
        assert_eq!(class_1.detection_count(), 1);
    }

    #[test]
    fn detection_boxes_as_arrays() {
        let boxes = vec![BoundingBox::new(0.1, 0.2, 0.3, 0.4, 0.9, 0)];
        let result = DetectionResult::new(1, boxes);
        let arrays = result.boxes_as_arrays();

        assert_eq!(arrays.len(), 1);
        assert!((arrays[0][0] - 0.1).abs() < 1e-6);
    }

    #[test]
    fn detection_nms() {
        // Two overlapping boxes, one with higher confidence
        let boxes = vec![
            BoundingBox::new(0.1, 0.1, 0.5, 0.5, 0.9, 0),
            BoundingBox::new(0.15, 0.15, 0.55, 0.55, 0.7, 0), // Overlaps significantly
            BoundingBox::new(0.8, 0.8, 0.95, 0.95, 0.8, 0),   // Separate
        ];
        let result = DetectionResult::new(1, boxes);

        let nms_result = result.nms(0.5);
        assert_eq!(nms_result.detection_count(), 2); // First and third kept
    }

    #[test]
    fn detection_best() {
        let boxes = vec![
            BoundingBox::new(0.1, 0.1, 0.2, 0.2, 0.5, 0),
            BoundingBox::new(0.3, 0.3, 0.4, 0.4, 0.9, 0),
            BoundingBox::new(0.5, 0.5, 0.6, 0.6, 0.7, 0),
        ];
        let result = DetectionResult::new(1, boxes);

        let best = result.best_detection();
        assert!(best.is_some());
        assert!((best.map_or(0.0, |b| b.confidence) - 0.9).abs() < 1e-6);
    }

    #[test]
    fn detection_serialization() {
        let result = DetectionResult::new(42, vec![BoundingBox::new(0.1, 0.2, 0.3, 0.4, 0.9, 0)]);

        let json = serde_json::to_string(&result);
        assert!(json.is_ok());

        let parsed: Result<DetectionResult, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default().frame_id, 42);
    }
}
