//! Manifest types for dataset runs.

use serde::{Deserialize, Serialize};
use std::path::PathBuf;

/// Schema version for manifest files.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub struct SchemaVersion {
    /// Major version.
    pub major: u32,
    /// Minor version.
    pub minor: u32,
}

impl SchemaVersion {
    /// Current schema version.
    pub const CURRENT: Self = Self { major: 1, minor: 0 };

    /// Creates a new schema version.
    #[must_use]
    pub const fn new(major: u32, minor: u32) -> Self {
        Self { major, minor }
    }

    /// Checks if this version is compatible with another.
    ///
    /// Compatible means same major version.
    #[must_use]
    pub const fn is_compatible(&self, other: &Self) -> bool {
        self.major == other.major
    }
}

impl std::fmt::Display for SchemaVersion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)
    }
}

/// Run manifest - metadata for a single data collection run.
///
/// Contains configuration and statistics for a training/validation run.
///
/// # Example
///
/// ```
/// use ml_types::{RunManifest, SchemaVersion};
///
/// let manifest = RunManifest {
///     schema_version: SchemaVersion::CURRENT,
///     run_id: "run_001".to_string(),
///     name: Some("Training Run 1".to_string()),
///     description: None,
///     created_at: 1704067200.0,
///     frame_count: 1000,
///     positive_count: 250,
///     image_width: 640,
///     image_height: 480,
///     path: None,
/// };
///
/// assert_eq!(manifest.positive_rate(), 0.25);
/// ```
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RunManifest {
    /// Schema version for compatibility.
    pub schema_version: SchemaVersion,

    /// Unique run identifier.
    pub run_id: String,

    /// Human-readable name (optional).
    pub name: Option<String>,

    /// Description (optional).
    pub description: Option<String>,

    /// Creation timestamp (Unix seconds).
    pub created_at: f64,

    /// Total number of frames.
    pub frame_count: u64,

    /// Number of positive frames (with labels).
    pub positive_count: u64,

    /// Image width.
    pub image_width: u32,

    /// Image height.
    pub image_height: u32,

    /// Path to run directory (optional, for loading).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<PathBuf>,
}

impl RunManifest {
    /// Creates a new run manifest.
    #[must_use]
    pub fn new(run_id: impl Into<String>, image_width: u32, image_height: u32) -> Self {
        Self {
            schema_version: SchemaVersion::CURRENT,
            run_id: run_id.into(),
            name: None,
            description: None,
            created_at: 0.0,
            frame_count: 0,
            positive_count: 0,
            image_width,
            image_height,
            path: None,
        }
    }

    /// Returns the positive sample rate (0.0 to 1.0).
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn positive_rate(&self) -> f64 {
        if self.frame_count == 0 {
            0.0
        } else {
            self.positive_count as f64 / self.frame_count as f64
        }
    }

    /// Returns the negative sample rate (0.0 to 1.0).
    #[must_use]
    pub fn negative_rate(&self) -> f64 {
        1.0 - self.positive_rate()
    }

    /// Checks if the run is empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.frame_count == 0
    }

    /// Returns the image dimensions as `(width, height)`.
    #[must_use]
    pub const fn image_size(&self) -> (u32, u32) {
        (self.image_width, self.image_height)
    }
}

impl Default for RunManifest {
    fn default() -> Self {
        Self::new("default", 640, 480)
    }
}

/// Dataset manifest - aggregates multiple runs.
///
/// Used for multi-run training datasets.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct DatasetManifest {
    /// Schema version.
    pub schema_version: SchemaVersion,

    /// Dataset identifier.
    pub dataset_id: String,

    /// Human-readable name.
    pub name: Option<String>,

    /// Description.
    pub description: Option<String>,

    /// Creation timestamp.
    pub created_at: f64,

    /// Individual run manifests.
    pub runs: Vec<RunManifest>,
}

impl DatasetManifest {
    /// Creates a new dataset manifest.
    #[must_use]
    pub fn new(dataset_id: impl Into<String>) -> Self {
        Self {
            schema_version: SchemaVersion::CURRENT,
            dataset_id: dataset_id.into(),
            name: None,
            description: None,
            created_at: 0.0,
            runs: Vec::new(),
        }
    }

    /// Adds a run to the dataset.
    pub fn add_run(&mut self, run: RunManifest) {
        self.runs.push(run);
    }

    /// Returns the total frame count across all runs.
    #[must_use]
    pub fn total_frame_count(&self) -> u64 {
        self.runs.iter().map(|r| r.frame_count).sum()
    }

    /// Returns the total positive count across all runs.
    #[must_use]
    pub fn total_positive_count(&self) -> u64 {
        self.runs.iter().map(|r| r.positive_count).sum()
    }

    /// Returns the overall positive rate.
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn positive_rate(&self) -> f64 {
        let total = self.total_frame_count();
        if total == 0 {
            0.0
        } else {
            self.total_positive_count() as f64 / total as f64
        }
    }

    /// Returns the number of runs.
    #[must_use]
    pub fn run_count(&self) -> usize {
        self.runs.len()
    }

    /// Checks if the dataset is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.runs.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schema_version() {
        let v = SchemaVersion::new(1, 2);
        assert_eq!(v.major, 1);
        assert_eq!(v.minor, 2);
        assert_eq!(format!("{v}"), "1.2");

        assert!(v.is_compatible(&SchemaVersion::new(1, 0)));
        assert!(!v.is_compatible(&SchemaVersion::new(2, 0)));
    }

    #[test]
    fn run_manifest_new() {
        let manifest = RunManifest::new("run_001", 640, 480);
        assert_eq!(manifest.run_id, "run_001");
        assert!(manifest.is_empty());
        assert_eq!(manifest.positive_rate(), 0.0);
    }

    #[test]
    fn run_manifest_positive_rate() {
        let mut manifest = RunManifest::new("test", 640, 480);
        manifest.frame_count = 100;
        manifest.positive_count = 25;

        assert!((manifest.positive_rate() - 0.25).abs() < 1e-10);
        assert!((manifest.negative_rate() - 0.75).abs() < 1e-10);
    }

    #[test]
    fn dataset_manifest_aggregation() {
        let mut dataset = DatasetManifest::new("dataset_001");

        let mut run1 = RunManifest::new("run_1", 640, 480);
        run1.frame_count = 100;
        run1.positive_count = 25;

        let mut run2 = RunManifest::new("run_2", 640, 480);
        run2.frame_count = 200;
        run2.positive_count = 50;

        dataset.add_run(run1);
        dataset.add_run(run2);

        assert_eq!(dataset.run_count(), 2);
        assert_eq!(dataset.total_frame_count(), 300);
        assert_eq!(dataset.total_positive_count(), 75);
        assert!((dataset.positive_rate() - 0.25).abs() < 1e-10);
    }

    #[test]
    fn manifest_serialization() {
        let manifest = RunManifest::new("test", 640, 480);
        let json = serde_json::to_string(&manifest);
        assert!(json.is_ok());

        let parsed: Result<RunManifest, _> = serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default().run_id, "test");
    }
}
