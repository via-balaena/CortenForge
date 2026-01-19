//! Image statistics for preprocessing.

use serde::{Deserialize, Serialize};

/// Image statistics for normalization.
///
/// Stores mean, standard deviation, and aspect ratio for preprocessing.
///
/// # Example
///
/// ```
/// use ml_types::ImageStats;
///
/// let stats = ImageStats {
///     mean: [0.485, 0.456, 0.406],
///     std: [0.229, 0.224, 0.225],
///     aspect_ratio: 1.333,  // 640/480
/// };
///
/// // Normalize a pixel value (channel 0)
/// let raw = 128.0 / 255.0;
/// let normalized = (raw - stats.mean[0]) / stats.std[0];
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ImageStats {
    /// Per-channel mean values (typically `[0, 1]`).
    pub mean: [f32; 3],

    /// Per-channel standard deviation values.
    pub std: [f32; 3],

    /// Image aspect ratio (width / height).
    pub aspect_ratio: f32,
}

impl ImageStats {
    /// Creates new image statistics.
    #[must_use]
    pub const fn new(mean: [f32; 3], std: [f32; 3], aspect_ratio: f32) -> Self {
        Self {
            mean,
            std,
            aspect_ratio,
        }
    }

    /// `ImageNet` normalization statistics.
    ///
    /// Standard values used for pretrained models.
    pub const IMAGENET: Self = Self {
        mean: [0.485, 0.456, 0.406],
        std: [0.229, 0.224, 0.225],
        aspect_ratio: 1.0,
    };

    /// Unity normalization (no normalization).
    ///
    /// Mean 0, std 1 - leaves values unchanged.
    pub const UNITY: Self = Self {
        mean: [0.0, 0.0, 0.0],
        std: [1.0, 1.0, 1.0],
        aspect_ratio: 1.0,
    };

    /// Creates statistics for zero-centered normalization.
    ///
    /// Scales to `[-1, 1]` range from `[0, 1]`.
    pub const ZERO_CENTERED: Self = Self {
        mean: [0.5, 0.5, 0.5],
        std: [0.5, 0.5, 0.5],
        aspect_ratio: 1.0,
    };

    /// Normalizes a single value.
    #[must_use]
    pub fn normalize(&self, value: f32, channel: usize) -> f32 {
        if channel >= 3 {
            return value;
        }
        (value - self.mean[channel]) / self.std[channel]
    }

    /// Denormalizes a single value.
    #[must_use]
    pub fn denormalize(&self, value: f32, channel: usize) -> f32 {
        if channel >= 3 {
            return value;
        }
        value.mul_add(self.std[channel], self.mean[channel])
    }

    /// Normalizes an RGB pixel.
    #[must_use]
    pub fn normalize_pixel(&self, rgb: [f32; 3]) -> [f32; 3] {
        [
            (rgb[0] - self.mean[0]) / self.std[0],
            (rgb[1] - self.mean[1]) / self.std[1],
            (rgb[2] - self.mean[2]) / self.std[2],
        ]
    }

    /// Denormalizes an RGB pixel.
    #[must_use]
    pub fn denormalize_pixel(&self, rgb: [f32; 3]) -> [f32; 3] {
        [
            rgb[0].mul_add(self.std[0], self.mean[0]),
            rgb[1].mul_add(self.std[1], self.mean[1]),
            rgb[2].mul_add(self.std[2], self.mean[2]),
        ]
    }

    /// Creates statistics with a specific aspect ratio.
    #[must_use]
    pub const fn with_aspect_ratio(mut self, aspect_ratio: f32) -> Self {
        self.aspect_ratio = aspect_ratio;
        self
    }

    /// Validates the statistics.
    ///
    /// Returns `false` if std contains zeros or `NaN` values.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.std.iter().all(|&s| s.is_finite() && s.abs() > 1e-10)
            && self.mean.iter().all(|m| m.is_finite())
            && self.aspect_ratio.is_finite()
            && self.aspect_ratio > 0.0
    }
}

impl Default for ImageStats {
    fn default() -> Self {
        Self::IMAGENET
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stats_new() {
        let stats = ImageStats::new([0.5, 0.5, 0.5], [0.5, 0.5, 0.5], 1.333);
        assert!((stats.aspect_ratio - 1.333).abs() < 1e-6);
    }

    #[test]
    fn stats_normalize() {
        let stats = ImageStats::ZERO_CENTERED;

        // 0.5 should become 0.0
        assert!(stats.normalize(0.5, 0).abs() < 1e-6);

        // 1.0 should become 1.0
        assert!((stats.normalize(1.0, 0) - 1.0).abs() < 1e-6);

        // 0.0 should become -1.0
        assert!((stats.normalize(0.0, 0) - (-1.0)).abs() < 1e-6);
    }

    #[test]
    fn stats_denormalize() {
        let stats = ImageStats::ZERO_CENTERED;

        // Round-trip
        let original = 0.75;
        let normalized = stats.normalize(original, 0);
        let restored = stats.denormalize(normalized, 0);
        assert!((restored - original).abs() < 1e-6);
    }

    #[test]
    fn stats_normalize_pixel() {
        let stats = ImageStats::UNITY;
        let pixel = [0.5, 0.6, 0.7];
        let normalized = stats.normalize_pixel(pixel);

        // With unity stats, values should be unchanged
        assert!((normalized[0] - 0.5).abs() < 1e-6);
        assert!((normalized[1] - 0.6).abs() < 1e-6);
        assert!((normalized[2] - 0.7).abs() < 1e-6);
    }

    #[test]
    fn stats_imagenet() {
        let stats = ImageStats::IMAGENET;
        assert!(stats.is_valid());
        assert!((stats.mean[0] - 0.485).abs() < 1e-6);
    }

    #[test]
    fn stats_validity() {
        let valid = ImageStats::IMAGENET;
        assert!(valid.is_valid());

        let zero_std = ImageStats::new([0.5, 0.5, 0.5], [0.0, 0.5, 0.5], 1.0);
        assert!(!zero_std.is_valid());

        let nan_mean = ImageStats::new([f32::NAN, 0.5, 0.5], [0.5, 0.5, 0.5], 1.0);
        assert!(!nan_mean.is_valid());

        let negative_aspect = ImageStats::new([0.5, 0.5, 0.5], [0.5, 0.5, 0.5], -1.0);
        assert!(!negative_aspect.is_valid());
    }

    #[test]
    fn stats_with_aspect() {
        let stats = ImageStats::IMAGENET.with_aspect_ratio(1.777);
        assert!((stats.aspect_ratio - 1.777).abs() < 1e-6);
        // Mean should be unchanged
        assert!((stats.mean[0] - 0.485).abs() < 1e-6);
    }

    #[test]
    fn stats_serialization() {
        let stats = ImageStats::IMAGENET;
        let json = serde_json::to_string(&stats);
        assert!(json.is_ok());

        let parsed: Result<ImageStats, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
    }
}
