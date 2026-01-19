//! Frame type for ML inference.

use serde::{Deserialize, Serialize};
use std::path::PathBuf;

/// A frame of image data ready for ML inference.
///
/// Contains image data with metadata, supporting both in-memory and
/// file-based workflows.
///
/// # Example
///
/// ```
/// use ml_types::Frame;
///
/// let frame = Frame {
///     id: 42,
///     timestamp: 1.5,
///     rgba: Some(vec![0u8; 640 * 480 * 4]),
///     size: (640, 480),
///     path: None,
/// };
///
/// assert_eq!(frame.id, 42);
/// assert_eq!(frame.pixel_count(), 640 * 480);
/// ```
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Frame {
    /// Unique frame identifier.
    pub id: u64,

    /// Timestamp in seconds (simulation or capture time).
    pub timestamp: f64,

    /// Optional raw RGBA8 data.
    ///
    /// Can be `None` when operating on file-based frames (lazy loading).
    pub rgba: Option<Vec<u8>>,

    /// Image dimensions: `(width, height)`.
    pub size: (u32, u32),

    /// Optional on-disk location for lazy loading.
    pub path: Option<PathBuf>,
}

impl Frame {
    /// Creates a new frame with in-memory image data.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn new(id: u64, timestamp: f64, rgba: Vec<u8>, width: u32, height: u32) -> Self {
        Self {
            id,
            timestamp,
            rgba: Some(rgba),
            size: (width, height),
            path: None,
        }
    }

    /// Creates a frame reference pointing to a file.
    #[must_use]
    #[allow(clippy::missing_const_for_fn)]
    pub fn from_path(id: u64, timestamp: f64, path: PathBuf, width: u32, height: u32) -> Self {
        Self {
            id,
            timestamp,
            rgba: None,
            size: (width, height),
            path: Some(path),
        }
    }

    /// Returns the image width.
    #[must_use]
    pub const fn width(&self) -> u32 {
        self.size.0
    }

    /// Returns the image height.
    #[must_use]
    pub const fn height(&self) -> u32 {
        self.size.1
    }

    /// Returns the total number of pixels.
    #[must_use]
    pub const fn pixel_count(&self) -> u32 {
        self.size.0 * self.size.1
    }

    /// Returns the expected RGBA buffer size.
    #[must_use]
    pub const fn expected_buffer_size(&self) -> usize {
        (self.size.0 * self.size.1 * 4) as usize
    }

    /// Checks if in-memory image data is present.
    #[must_use]
    pub const fn has_data(&self) -> bool {
        self.rgba.is_some()
    }

    /// Checks if a file path is present.
    #[must_use]
    pub const fn has_path(&self) -> bool {
        self.path.is_some()
    }

    /// Checks if the RGBA buffer has the expected size.
    #[must_use]
    pub fn has_valid_buffer_size(&self) -> bool {
        self.rgba
            .as_ref()
            .is_none_or(|data| data.len() == self.expected_buffer_size())
    }

    /// Returns the aspect ratio (width / height).
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn aspect_ratio(&self) -> f64 {
        f64::from(self.size.0) / f64::from(self.size.1)
    }
}

impl Default for Frame {
    fn default() -> Self {
        Self {
            id: 0,
            timestamp: 0.0,
            rgba: None,
            size: (0, 0),
            path: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_new() {
        let frame = Frame::new(1, 0.5, vec![0u8; 640 * 480 * 4], 640, 480);
        assert_eq!(frame.id, 1);
        assert!((frame.timestamp - 0.5).abs() < 1e-10);
        assert!(frame.has_data());
        assert!(!frame.has_path());
    }

    #[test]
    fn frame_from_path() {
        let frame = Frame::from_path(2, 1.0, PathBuf::from("/tmp/test.png"), 1920, 1080);
        assert_eq!(frame.id, 2);
        assert!(!frame.has_data());
        assert!(frame.has_path());
    }

    #[test]
    fn frame_dimensions() {
        let frame = Frame::new(0, 0.0, vec![0u8; 100 * 50 * 4], 100, 50);
        assert_eq!(frame.width(), 100);
        assert_eq!(frame.height(), 50);
        assert_eq!(frame.pixel_count(), 5000);
        assert_eq!(frame.expected_buffer_size(), 20000);
    }

    #[test]
    fn frame_buffer_validation() {
        let good = Frame::new(0, 0.0, vec![0u8; 640 * 480 * 4], 640, 480);
        assert!(good.has_valid_buffer_size());

        let bad = Frame {
            id: 0,
            timestamp: 0.0,
            rgba: Some(vec![0u8; 100]), // Wrong size
            size: (640, 480),
            path: None,
        };
        assert!(!bad.has_valid_buffer_size());

        let no_data = Frame::from_path(0, 0.0, PathBuf::from("/tmp/x.png"), 640, 480);
        assert!(no_data.has_valid_buffer_size()); // None is valid
    }

    #[test]
    fn frame_aspect_ratio() {
        let frame = Frame::new(0, 0.0, vec![0u8; 1920 * 1080 * 4], 1920, 1080);
        let expected = 1920.0 / 1080.0;
        assert!((frame.aspect_ratio() - expected).abs() < 1e-10);
    }

    #[test]
    fn frame_serialization() {
        let frame = Frame::new(1, 0.5, vec![1, 2, 3, 4], 1, 1);
        let json = serde_json::to_string(&frame);
        assert!(json.is_ok());

        let parsed: Result<Frame, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default().id, 1);
    }
}
