//! Dataset sample types.

use serde::{Deserialize, Serialize};

/// A single dataset sample for ML training.
///
/// Contains the image data (in CHW layout) and associated labels.
///
/// # Image Format
///
/// The image is stored as a flat `Vec<f32>` in CHW (Channel-Height-Width)
/// layout, normalized to `[0, 1]`. This is the standard format for
/// neural network input.
///
/// # Example
///
/// ```
/// use ml_dataset::DatasetSample;
///
/// let sample = DatasetSample::new(
///     42,
///     vec![0.5; 3 * 64 * 64], // 64x64 RGB image
///     64, 64,
///     vec![[0.1, 0.2, 0.3, 0.4]], // One bounding box
/// );
///
/// assert_eq!(sample.frame_id, 42);
/// assert_eq!(sample.num_boxes(), 1);
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DatasetSample {
    /// Frame/sample ID.
    pub frame_id: u64,

    /// Image data in CHW layout, normalized to `[0, 1]`.
    pub image_chw: Vec<f32>,

    /// Image width in pixels.
    pub width: u32,

    /// Image height in pixels.
    pub height: u32,

    /// Normalized bounding boxes `[[x0, y0, x1, y1], ...]`.
    pub boxes: Vec<[f32; 4]>,
}

impl DatasetSample {
    /// Creates a new dataset sample.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn new(
        frame_id: u64,
        image_chw: Vec<f32>,
        width: u32,
        height: u32,
        boxes: Vec<[f32; 4]>,
    ) -> Self {
        Self {
            frame_id,
            image_chw,
            width,
            height,
            boxes,
        }
    }

    /// Creates an empty sample with just an ID.
    ///
    /// Useful for testing or as a placeholder.
    #[must_use]
    pub const fn empty(frame_id: u64) -> Self {
        Self {
            frame_id,
            image_chw: Vec::new(),
            width: 0,
            height: 0,
            boxes: Vec::new(),
        }
    }

    /// Returns the number of bounding boxes.
    #[must_use]
    pub fn num_boxes(&self) -> usize {
        self.boxes.len()
    }

    /// Returns `true` if the sample has any bounding boxes.
    #[must_use]
    pub fn has_boxes(&self) -> bool {
        !self.boxes.is_empty()
    }

    /// Returns the expected image data length (C * H * W).
    #[must_use]
    pub const fn expected_image_len(&self, channels: u32) -> usize {
        (channels as usize) * (self.height as usize) * (self.width as usize)
    }

    /// Validates the sample data.
    ///
    /// Returns `true` if:
    /// - Image dimensions are positive
    /// - Image data length matches dimensions
    /// - All boxes have valid coordinates
    #[must_use]
    pub fn is_valid(&self) -> bool {
        if self.width == 0 || self.height == 0 {
            return self.image_chw.is_empty() && self.boxes.is_empty();
        }

        let expected_len = self.expected_image_len(3); // RGB
        if self.image_chw.len() != expected_len {
            return false;
        }

        // Check boxes
        for bbox in &self.boxes {
            let [x0, y0, x1, y1] = bbox;
            if !(0.0..=1.0).contains(x0)
                || !(0.0..=1.0).contains(y0)
                || !(0.0..=1.0).contains(x1)
                || !(0.0..=1.0).contains(y1)
            {
                return false;
            }
            if x0 > x1 || y0 > y1 {
                return false;
            }
        }

        true
    }
}

impl Default for DatasetSample {
    fn default() -> Self {
        Self::empty(0)
    }
}

/// Index for locating a sample within a dataset.
///
/// # Example
///
/// ```
/// use ml_dataset::SampleIndex;
///
/// let idx = SampleIndex::new("run_001", 42);
/// assert_eq!(idx.run_id, "run_001");
/// assert_eq!(idx.frame_id, 42);
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SampleIndex {
    /// Run identifier (e.g., `run_001`).
    pub run_id: String,

    /// Frame ID within the run.
    pub frame_id: u64,
}

impl SampleIndex {
    /// Creates a new sample index.
    #[must_use]
    pub fn new(run_id: impl Into<String>, frame_id: u64) -> Self {
        Self {
            run_id: run_id.into(),
            frame_id,
        }
    }
}

/// Mode for resizing images during preprocessing.
///
/// # Example
///
/// ```
/// use ml_dataset::ResizeMode;
///
/// let mode = ResizeMode::default();
/// assert!(matches!(mode, ResizeMode::Stretch));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum ResizeMode {
    /// Stretch to target size (may distort aspect ratio).
    #[default]
    Stretch,

    /// Resize preserving aspect ratio, pad with black.
    Letterbox,

    /// Resize preserving aspect ratio, crop excess.
    CropCenter,

    /// No resizing, use original dimensions.
    None,
}

impl ResizeMode {
    /// Returns the mode name.
    #[must_use]
    pub const fn name(&self) -> &'static str {
        match self {
            Self::Stretch => "stretch",
            Self::Letterbox => "letterbox",
            Self::CropCenter => "crop_center",
            Self::None => "none",
        }
    }
}

impl std::fmt::Display for ResizeMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_new() {
        let sample = DatasetSample::new(1, vec![0.5; 12], 2, 2, vec![[0.1, 0.2, 0.3, 0.4]]);

        assert_eq!(sample.frame_id, 1);
        assert_eq!(sample.width, 2);
        assert_eq!(sample.height, 2);
        assert_eq!(sample.num_boxes(), 1);
        assert!(sample.has_boxes());
    }

    #[test]
    fn sample_empty() {
        let sample = DatasetSample::empty(42);
        assert_eq!(sample.frame_id, 42);
        assert!(!sample.has_boxes());
        assert!(sample.is_valid());
    }

    #[test]
    fn sample_expected_image_len() {
        let sample = DatasetSample::empty(0);
        let sample = DatasetSample {
            width: 64,
            height: 48,
            ..sample
        };

        assert_eq!(sample.expected_image_len(3), 3 * 64 * 48);
        assert_eq!(sample.expected_image_len(1), 64 * 48);
    }

    #[test]
    fn sample_is_valid() {
        // Valid sample
        let valid = DatasetSample::new(0, vec![0.5; 3 * 4 * 4], 4, 4, vec![[0.1, 0.2, 0.3, 0.4]]);
        assert!(valid.is_valid());

        // Invalid: wrong image length
        let bad_len = DatasetSample::new(0, vec![0.5; 10], 4, 4, vec![]);
        assert!(!bad_len.is_valid());

        // Invalid: box out of range
        let bad_box = DatasetSample::new(0, vec![0.5; 48], 4, 4, vec![[-0.1, 0.2, 0.3, 0.4]]);
        assert!(!bad_box.is_valid());

        // Invalid: box coordinates reversed
        let reversed = DatasetSample::new(0, vec![0.5; 48], 4, 4, vec![[0.5, 0.2, 0.3, 0.4]]);
        assert!(!reversed.is_valid());
    }

    #[test]
    fn sample_serialization() {
        let sample = DatasetSample::new(1, vec![0.5; 12], 2, 2, vec![[0.1, 0.2, 0.3, 0.4]]);
        let json = serde_json::to_string(&sample);
        assert!(json.is_ok());

        let parsed: std::result::Result<DatasetSample, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }

    #[test]
    fn sample_index_new() {
        let idx = SampleIndex::new("run_001", 42);
        assert_eq!(idx.run_id, "run_001");
        assert_eq!(idx.frame_id, 42);
    }

    #[test]
    fn resize_mode_default() {
        let mode = ResizeMode::default();
        assert!(matches!(mode, ResizeMode::Stretch));
    }

    #[test]
    fn resize_mode_name() {
        assert_eq!(ResizeMode::Stretch.name(), "stretch");
        assert_eq!(ResizeMode::Letterbox.name(), "letterbox");
        assert_eq!(ResizeMode::CropCenter.name(), "crop_center");
        assert_eq!(ResizeMode::None.name(), "none");
    }

    #[test]
    fn resize_mode_display() {
        assert_eq!(format!("{}", ResizeMode::Letterbox), "letterbox");
    }

    #[test]
    fn resize_mode_serialization() {
        let mode = ResizeMode::Letterbox;
        let json = serde_json::to_string(&mode);
        assert!(json.is_ok());

        let parsed: std::result::Result<ResizeMode, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), mode);
    }
}
