//! Dataset lifecycle management for CortenForge.
//!
//! This crate provides tools for working with ML datasets:
//!
//! # Dataset Operations
//!
//! - [`DatasetSample`] - Single training sample with image and labels
//! - [`split_dataset`] - Split dataset into train/validation sets
//! - [`DatasetSummary`] - Statistics about a dataset
//!
//! # Warehouse (Sharded Storage)
//!
//! - [`ShardMetadata`] - Metadata for a data shard
//! - [`WarehouseManifest`] - Index of all shards
//!
//! # Data Loading
//!
//! - [`load_sample`] - Load a single sample from disk
//! - [`index_runs`] - Index available dataset runs
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - Training pipelines
//! - Data preprocessing scripts
//! - Dataset validation tools
//! - ETL workflows
//!
//! # Example
//!
//! ```
//! use ml_dataset::{DatasetSample, split_dataset, SplitRatio};
//!
//! // Create some samples
//! let samples: Vec<DatasetSample> = vec![
//!     DatasetSample::empty(0),
//!     DatasetSample::empty(1),
//!     DatasetSample::empty(2),
//!     DatasetSample::empty(3),
//! ];
//!
//! // Split into train/val
//! let ratio = SplitRatio::new(0.75);
//! let (train, val) = split_dataset(&samples, ratio, Some(42));
//!
//! assert_eq!(train.len(), 3);
//! assert_eq!(val.len(), 1);
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

mod error;
mod sample;
mod splits;
mod summary;
mod warehouse;

// Re-export sample types
pub use sample::{DatasetSample, ResizeMode, SampleIndex};

// Re-export split utilities
pub use splits::{SplitRatio, split_dataset, split_stratified};

// Re-export summary types
pub use summary::DatasetSummary;

// Re-export warehouse types
pub use warehouse::{ShardBuilder, ShardMetadata, WarehouseManifest};

// Re-export error types
pub use error::{DatasetError, Result};

/// Prelude for convenient imports.
pub mod prelude {
    pub use super::{
        DatasetError, DatasetSample, DatasetSummary, ResizeMode, SampleIndex, ShardBuilder,
        ShardMetadata, SplitRatio, WarehouseManifest, split_dataset, split_stratified,
    };
}
