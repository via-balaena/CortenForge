//! Metadata types for capture and dataset storage.

use serde::{Deserialize, Serialize};

use crate::{DetectionLabel, ValidationError};

/// Per-frame capture metadata.
///
/// Contains all information needed to reconstruct or validate a captured frame.
///
/// # Example
///
/// ```
/// use ml_types::{CaptureMetadata, DetectionLabel, LabelSource};
///
/// let metadata = CaptureMetadata {
///     frame_id: 42,
///     sim_time: 1.5,
///     unix_time: 1704067200.5,
///     image: "frame_0042.png".to_string(),
///     image_present: true,
///     camera_active: true,
///     label_seed: 12345,
///     labels: vec![DetectionLabel::with_bbox_norm(
///         [0.0, 0.0, 1.0],
///         [0.1, 0.2, 0.3, 0.4],
///         0,
///         LabelSource::SimAuto,
///     )],
/// };
///
/// assert!(metadata.validate().is_ok());
/// ```
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CaptureMetadata {
    /// Unique frame identifier.
    pub frame_id: u64,

    /// Simulation time in seconds.
    pub sim_time: f64,

    /// Unix timestamp (seconds since epoch).
    pub unix_time: f64,

    /// Image filename (relative to run directory).
    pub image: String,

    /// Whether the image file is present on disk.
    pub image_present: bool,

    /// Whether the camera was active during capture.
    pub camera_active: bool,

    /// Random seed used for label generation.
    pub label_seed: u64,

    /// Ground truth labels for this frame.
    pub labels: Vec<DetectionLabel>,
}

impl CaptureMetadata {
    /// Creates empty capture metadata.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn new(frame_id: u64) -> Self {
        Self {
            frame_id,
            sim_time: 0.0,
            unix_time: 0.0,
            image: String::new(),
            image_present: false,
            camera_active: false,
            label_seed: 0,
            labels: Vec::new(),
        }
    }

    /// Validates the metadata.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Image is present but path is empty
    /// - Any label fails validation
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.image_present && self.image.trim().is_empty() {
            return Err(ValidationError::MissingImage);
        }

        for label in &self.labels {
            label.validate()?;
        }

        Ok(())
    }

    /// Returns the number of labels.
    #[must_use]
    pub fn label_count(&self) -> usize {
        self.labels.len()
    }

    /// Checks if this frame has any labels.
    #[must_use]
    pub fn has_labels(&self) -> bool {
        !self.labels.is_empty()
    }

    /// Checks if this is a positive sample (has at least one label).
    #[must_use]
    pub fn is_positive(&self) -> bool {
        !self.labels.is_empty()
    }
}

impl Default for CaptureMetadata {
    fn default() -> Self {
        Self::new(0)
    }
}

/// Lightweight frame metadata (subset of `CaptureMetadata`).
///
/// Used for quick indexing and filtering without loading full labels.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct FrameMetadata {
    /// Unique frame identifier.
    pub frame_id: u64,

    /// Timestamp in seconds.
    pub timestamp: f64,

    /// Number of labels (for stratified sampling).
    pub label_count: u32,

    /// Whether the camera was active.
    pub camera_active: bool,
}

impl FrameMetadata {
    /// Creates new frame metadata.
    #[must_use]
    pub const fn new(frame_id: u64, timestamp: f64, label_count: u32, camera_active: bool) -> Self {
        Self {
            frame_id,
            timestamp,
            label_count,
            camera_active,
        }
    }

    /// Creates from capture metadata.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn from_capture(capture: &CaptureMetadata) -> Self {
        Self {
            frame_id: capture.frame_id,
            timestamp: capture.sim_time,
            label_count: capture.labels.len() as u32,
            camera_active: capture.camera_active,
        }
    }

    /// Checks if this is a positive sample.
    #[must_use]
    pub const fn is_positive(&self) -> bool {
        self.label_count > 0
    }
}

impl Default for FrameMetadata {
    fn default() -> Self {
        Self::new(0, 0.0, 0, false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LabelSource;

    #[test]
    fn capture_metadata_new() {
        let meta = CaptureMetadata::new(42);
        assert_eq!(meta.frame_id, 42);
        assert!(!meta.has_labels());
        assert!(!meta.is_positive());
    }

    #[test]
    fn capture_metadata_validate_ok() {
        let meta = CaptureMetadata {
            frame_id: 1,
            sim_time: 1.0,
            unix_time: 1000.0,
            image: "test.png".to_string(),
            image_present: true,
            camera_active: true,
            label_seed: 0,
            labels: vec![],
        };
        assert!(meta.validate().is_ok());
    }

    #[test]
    fn capture_metadata_validate_missing_image() {
        let meta = CaptureMetadata {
            frame_id: 1,
            sim_time: 1.0,
            unix_time: 1000.0,
            image: "  ".to_string(), // Whitespace only
            image_present: true,
            camera_active: true,
            label_seed: 0,
            labels: vec![],
        };
        assert!(matches!(
            meta.validate(),
            Err(ValidationError::MissingImage)
        ));
    }

    #[test]
    fn capture_metadata_with_labels() {
        let meta = CaptureMetadata {
            frame_id: 1,
            sim_time: 1.0,
            unix_time: 1000.0,
            image: "test.png".to_string(),
            image_present: true,
            camera_active: true,
            label_seed: 0,
            labels: vec![DetectionLabel::with_bbox_norm(
                [0.0, 0.0, 0.0],
                [0.1, 0.2, 0.3, 0.4],
                0,
                LabelSource::SimAuto,
            )],
        };
        assert_eq!(meta.label_count(), 1);
        assert!(meta.is_positive());
        assert!(meta.validate().is_ok());
    }

    #[test]
    fn frame_metadata_from_capture() {
        let capture = CaptureMetadata {
            frame_id: 42,
            sim_time: 1.5,
            unix_time: 1000.0,
            image: String::new(),
            image_present: false,
            camera_active: true,
            label_seed: 0,
            labels: vec![DetectionLabel::default(), DetectionLabel::default()],
        };

        let frame = FrameMetadata::from_capture(&capture);
        assert_eq!(frame.frame_id, 42);
        assert!((frame.timestamp - 1.5).abs() < 1e-10);
        assert_eq!(frame.label_count, 2);
        assert!(frame.is_positive());
    }

    #[test]
    fn metadata_serialization() {
        let capture = CaptureMetadata::new(1);
        let json = serde_json::to_string(&capture);
        assert!(json.is_ok());

        let frame = FrameMetadata::new(1, 0.5, 2, true);
        let json = serde_json::to_string(&frame);
        assert!(json.is_ok());
    }
}
