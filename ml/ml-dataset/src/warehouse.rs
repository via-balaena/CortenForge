//! Dataset warehouse for sharded storage.
//!
//! The warehouse system enables efficient storage and retrieval of large
//! datasets by splitting them into shards. Each shard contains a subset
//! of samples and can be loaded independently.

use std::path::PathBuf;

use serde::{Deserialize, Serialize};

use crate::error::{DatasetError, Result};
use crate::sample::DatasetSample;
use crate::summary::DatasetSummary;

/// Metadata for a single shard in the warehouse.
///
/// # Example
///
/// ```
/// use ml_dataset::ShardMetadata;
///
/// let shard = ShardMetadata::new(0, 1000, "shard_000.bin");
/// assert_eq!(shard.index, 0);
/// assert_eq!(shard.sample_count, 1000);
/// ```
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ShardMetadata {
    /// Shard index (0-based).
    pub index: usize,

    /// Number of samples in this shard.
    pub sample_count: usize,

    /// Relative path to shard file.
    pub path: PathBuf,

    /// Optional checksum for integrity verification.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub checksum: Option<String>,

    /// Size in bytes.
    #[serde(default)]
    pub size_bytes: u64,
}

impl ShardMetadata {
    /// Creates new shard metadata.
    #[must_use]
    pub fn new(index: usize, sample_count: usize, path: impl Into<PathBuf>) -> Self {
        Self {
            index,
            sample_count,
            path: path.into(),
            checksum: None,
            size_bytes: 0,
        }
    }

    /// Sets the checksum.
    #[must_use]
    pub fn with_checksum(mut self, checksum: impl Into<String>) -> Self {
        self.checksum = Some(checksum.into());
        self
    }

    /// Sets the size in bytes.
    #[must_use]
    pub const fn with_size(mut self, size_bytes: u64) -> Self {
        self.size_bytes = size_bytes;
        self
    }

    /// Returns true if this shard is empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.sample_count == 0
    }
}

/// Manifest for a dataset warehouse.
///
/// The manifest tracks all shards in a warehouse and provides
/// methods for loading and querying the dataset.
///
/// # Example
///
/// ```
/// use ml_dataset::WarehouseManifest;
///
/// let manifest = WarehouseManifest::new("my_dataset", "1.0.0");
/// assert_eq!(manifest.name, "my_dataset");
/// assert!(manifest.shards.is_empty());
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WarehouseManifest {
    /// Dataset name.
    pub name: String,

    /// Dataset version.
    pub version: String,

    /// List of shards.
    pub shards: Vec<ShardMetadata>,

    /// Total sample count across all shards.
    pub total_samples: usize,

    /// Optional dataset summary.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub summary: Option<DatasetSummary>,

    /// Creation timestamp (Unix epoch seconds).
    #[serde(default)]
    pub created_at: u64,

    /// Optional description.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    /// Optional tags for categorization.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tags: Vec<String>,
}

impl WarehouseManifest {
    /// Creates a new empty manifest.
    #[must_use]
    pub fn new(name: impl Into<String>, version: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            version: version.into(),
            shards: Vec::new(),
            total_samples: 0,
            summary: None,
            created_at: 0,
            description: None,
            tags: Vec::new(),
        }
    }

    /// Sets the description.
    #[must_use]
    pub fn with_description(mut self, description: impl Into<String>) -> Self {
        self.description = Some(description.into());
        self
    }

    /// Adds a tag.
    #[must_use]
    pub fn with_tag(mut self, tag: impl Into<String>) -> Self {
        self.tags.push(tag.into());
        self
    }

    /// Sets the creation timestamp.
    #[must_use]
    pub const fn with_created_at(mut self, timestamp: u64) -> Self {
        self.created_at = timestamp;
        self
    }

    /// Sets the summary.
    #[must_use]
    pub fn with_summary(mut self, summary: DatasetSummary) -> Self {
        self.summary = Some(summary);
        self
    }

    /// Adds a shard to the manifest.
    pub fn add_shard(&mut self, shard: ShardMetadata) {
        self.total_samples += shard.sample_count;
        self.shards.push(shard);
    }

    /// Returns the number of shards.
    #[must_use]
    pub fn shard_count(&self) -> usize {
        self.shards.len()
    }

    /// Returns true if the warehouse is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.shards.is_empty()
    }

    /// Gets shard metadata by index.
    #[must_use]
    pub fn get_shard(&self, index: usize) -> Option<&ShardMetadata> {
        self.shards.get(index)
    }

    /// Computes the shard index and local offset for a global sample index.
    ///
    /// Returns `None` if the index is out of bounds.
    #[must_use]
    pub fn locate_sample(&self, global_index: usize) -> Option<(usize, usize)> {
        let mut offset = 0;
        for (shard_idx, shard) in self.shards.iter().enumerate() {
            if global_index < offset + shard.sample_count {
                return Some((shard_idx, global_index - offset));
            }
            offset += shard.sample_count;
        }
        None
    }

    /// Validates the manifest structure.
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Name or version is empty
    /// - Total samples doesn't match sum of shard samples
    /// - Shard indices are not sequential
    pub fn validate(&self) -> Result<()> {
        if self.name.is_empty() {
            return Err(DatasetError::validation("name cannot be empty"));
        }

        if self.version.is_empty() {
            return Err(DatasetError::validation("version cannot be empty"));
        }

        // Verify total matches shards
        let computed_total: usize = self.shards.iter().map(|s| s.sample_count).sum();
        if computed_total != self.total_samples {
            return Err(DatasetError::validation(format!(
                "total_samples mismatch: manifest says {}, shards sum to {}",
                self.total_samples, computed_total
            )));
        }

        // Verify shard indices are sequential
        for (expected, shard) in self.shards.iter().enumerate() {
            if shard.index != expected {
                return Err(DatasetError::validation(format!(
                    "shard index mismatch: expected {}, got {}",
                    expected, shard.index
                )));
            }
        }

        Ok(())
    }

    /// Serializes the manifest to JSON.
    ///
    /// # Errors
    ///
    /// Returns an error if serialization fails.
    pub fn to_json(&self) -> Result<String> {
        serde_json::to_string_pretty(self).map_err(DatasetError::from)
    }

    /// Deserializes a manifest from JSON.
    ///
    /// # Errors
    ///
    /// Returns an error if deserialization fails.
    pub fn from_json(json: &str) -> Result<Self> {
        serde_json::from_str(json).map_err(DatasetError::from)
    }
}

/// Builder for creating warehouse shards.
///
/// Helps split a dataset into fixed-size shards for storage.
///
/// # Example
///
/// ```
/// use ml_dataset::{DatasetSample, ShardBuilder};
///
/// let mut builder = ShardBuilder::new(100); // 100 samples per shard
///
/// for i in 0..250 {
///     builder.add_sample(DatasetSample::empty(i));
/// }
///
/// let shards = builder.finish();
/// assert_eq!(shards.len(), 3); // 100 + 100 + 50
/// ```
#[derive(Debug)]
pub struct ShardBuilder {
    samples_per_shard: usize,
    current_shard: Vec<DatasetSample>,
    completed_shards: Vec<Vec<DatasetSample>>,
}

impl ShardBuilder {
    /// Creates a new shard builder.
    ///
    /// # Arguments
    ///
    /// - `samples_per_shard`: Maximum samples per shard
    #[must_use]
    pub fn new(samples_per_shard: usize) -> Self {
        Self {
            samples_per_shard: samples_per_shard.max(1),
            current_shard: Vec::new(),
            completed_shards: Vec::new(),
        }
    }

    /// Adds a sample to the builder.
    pub fn add_sample(&mut self, sample: DatasetSample) {
        self.current_shard.push(sample);

        if self.current_shard.len() >= self.samples_per_shard {
            let shard = std::mem::take(&mut self.current_shard);
            self.completed_shards.push(shard);
        }
    }

    /// Returns the number of completed shards.
    #[must_use]
    pub fn completed_count(&self) -> usize {
        self.completed_shards.len()
    }

    /// Returns the number of samples in the current (incomplete) shard.
    #[must_use]
    pub fn current_count(&self) -> usize {
        self.current_shard.len()
    }

    /// Finishes building and returns all shards.
    ///
    /// Includes the current partial shard if non-empty.
    #[must_use]
    pub fn finish(mut self) -> Vec<Vec<DatasetSample>> {
        if !self.current_shard.is_empty() {
            self.completed_shards.push(self.current_shard);
        }
        self.completed_shards
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shard_metadata_new() {
        let shard = ShardMetadata::new(0, 1000, "shard_000.bin");
        assert_eq!(shard.index, 0);
        assert_eq!(shard.sample_count, 1000);
        assert_eq!(shard.path, PathBuf::from("shard_000.bin"));
        assert!(shard.checksum.is_none());
        assert!(!shard.is_empty());
    }

    #[test]
    fn shard_metadata_with_checksum() {
        let shard = ShardMetadata::new(0, 100, "shard.bin").with_checksum("abc123");
        assert_eq!(shard.checksum, Some("abc123".to_string()));
    }

    #[test]
    fn shard_metadata_with_size() {
        let shard = ShardMetadata::new(0, 100, "shard.bin").with_size(1024);
        assert_eq!(shard.size_bytes, 1024);
    }

    #[test]
    fn shard_metadata_empty() {
        let shard = ShardMetadata::new(0, 0, "empty.bin");
        assert!(shard.is_empty());
    }

    #[test]
    fn manifest_new() {
        let manifest = WarehouseManifest::new("test_dataset", "1.0.0");
        assert_eq!(manifest.name, "test_dataset");
        assert_eq!(manifest.version, "1.0.0");
        assert!(manifest.shards.is_empty());
        assert_eq!(manifest.total_samples, 0);
        assert!(manifest.is_empty());
    }

    #[test]
    fn manifest_with_description() {
        let manifest = WarehouseManifest::new("test", "1.0").with_description("A test dataset");
        assert_eq!(manifest.description, Some("A test dataset".to_string()));
    }

    #[test]
    fn manifest_with_tags() {
        let manifest = WarehouseManifest::new("test", "1.0")
            .with_tag("detection")
            .with_tag("synthetic");
        assert_eq!(manifest.tags, vec!["detection", "synthetic"]);
    }

    #[test]
    fn manifest_add_shard() {
        let mut manifest = WarehouseManifest::new("test", "1.0");
        manifest.add_shard(ShardMetadata::new(0, 1000, "shard_000.bin"));
        manifest.add_shard(ShardMetadata::new(1, 500, "shard_001.bin"));

        assert_eq!(manifest.shard_count(), 2);
        assert_eq!(manifest.total_samples, 1500);
        assert!(!manifest.is_empty());
    }

    #[test]
    fn manifest_get_shard() {
        let mut manifest = WarehouseManifest::new("test", "1.0");
        manifest.add_shard(ShardMetadata::new(0, 100, "shard.bin"));

        assert!(manifest.get_shard(0).is_some());
        assert!(manifest.get_shard(1).is_none());
    }

    #[test]
    fn manifest_locate_sample() {
        let mut manifest = WarehouseManifest::new("test", "1.0");
        manifest.add_shard(ShardMetadata::new(0, 100, "shard_0.bin"));
        manifest.add_shard(ShardMetadata::new(1, 200, "shard_1.bin"));
        manifest.add_shard(ShardMetadata::new(2, 50, "shard_2.bin"));

        // First shard
        assert_eq!(manifest.locate_sample(0), Some((0, 0)));
        assert_eq!(manifest.locate_sample(99), Some((0, 99)));

        // Second shard
        assert_eq!(manifest.locate_sample(100), Some((1, 0)));
        assert_eq!(manifest.locate_sample(299), Some((1, 199)));

        // Third shard
        assert_eq!(manifest.locate_sample(300), Some((2, 0)));
        assert_eq!(manifest.locate_sample(349), Some((2, 49)));

        // Out of bounds
        assert_eq!(manifest.locate_sample(350), None);
    }

    #[test]
    fn manifest_validate_success() {
        let mut manifest = WarehouseManifest::new("test", "1.0");
        manifest.add_shard(ShardMetadata::new(0, 100, "shard_0.bin"));
        manifest.add_shard(ShardMetadata::new(1, 200, "shard_1.bin"));

        assert!(manifest.validate().is_ok());
    }

    #[test]
    fn manifest_validate_empty_name() {
        let manifest = WarehouseManifest::new("", "1.0");
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn manifest_validate_empty_version() {
        let manifest = WarehouseManifest::new("test", "");
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn manifest_validate_total_mismatch() {
        let mut manifest = WarehouseManifest::new("test", "1.0");
        manifest.add_shard(ShardMetadata::new(0, 100, "shard.bin"));
        manifest.total_samples = 999; // Wrong!

        assert!(manifest.validate().is_err());
    }

    #[test]
    fn manifest_validate_index_mismatch() {
        let mut manifest = WarehouseManifest::new("test", "1.0");
        manifest
            .shards
            .push(ShardMetadata::new(0, 100, "shard_0.bin"));
        manifest
            .shards
            .push(ShardMetadata::new(5, 100, "shard_5.bin")); // Wrong index!
        manifest.total_samples = 200;

        assert!(manifest.validate().is_err());
    }

    #[test]
    fn manifest_serialization() {
        let mut manifest = WarehouseManifest::new("test", "1.0")
            .with_description("Test")
            .with_created_at(1234567890);
        manifest.add_shard(ShardMetadata::new(0, 100, "shard.bin"));

        let json = manifest.to_json();
        assert!(json.is_ok());

        let parsed = WarehouseManifest::from_json(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_else(|_| manifest.clone()), manifest);
    }

    #[test]
    fn shard_builder_exact_fit() {
        let mut builder = ShardBuilder::new(100);

        for i in 0..200 {
            builder.add_sample(DatasetSample::empty(i));
        }

        let shards = builder.finish();
        assert_eq!(shards.len(), 2);
        assert_eq!(shards[0].len(), 100);
        assert_eq!(shards[1].len(), 100);
    }

    #[test]
    fn shard_builder_partial_shard() {
        let mut builder = ShardBuilder::new(100);

        for i in 0..150 {
            builder.add_sample(DatasetSample::empty(i));
        }

        assert_eq!(builder.completed_count(), 1);
        assert_eq!(builder.current_count(), 50);

        let shards = builder.finish();
        assert_eq!(shards.len(), 2);
        assert_eq!(shards[0].len(), 100);
        assert_eq!(shards[1].len(), 50);
    }

    #[test]
    fn shard_builder_empty() {
        let builder = ShardBuilder::new(100);
        let shards = builder.finish();
        assert!(shards.is_empty());
    }

    #[test]
    fn shard_builder_min_size() {
        // Should enforce minimum of 1
        let mut builder = ShardBuilder::new(0);
        builder.add_sample(DatasetSample::empty(0));
        builder.add_sample(DatasetSample::empty(1));

        let shards = builder.finish();
        assert_eq!(shards.len(), 2); // Each sample is its own shard
    }
}
