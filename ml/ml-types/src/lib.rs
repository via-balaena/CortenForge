//! ML types for CortenForge.
//!
//! This crate provides types for machine learning workflows:
//!
//! # Inference Types
//!
//! - [`Frame`] - Processed image ready for ML inference
//! - [`DetectionResult`] - Output from object detection models
//! - [`BoundingBox`] - Normalized bounding box with confidence
//! - [`SegmentationMask`] - Per-pixel class labels
//! - [`KeypointSet`] - Body/object keypoints with skeleton
//!
//! # Dataset Schema Types
//!
//! - [`DetectionLabel`] - Ground truth bounding box with class
//! - [`LabelSource`] - Provenance: `SimAuto`, `Human`, or `Model`
//! - [`CaptureMetadata`] - Per-frame capture metadata
//! - [`RunManifest`] - Dataset run configuration
//! - [`DatasetManifest`] - Multi-run dataset metadata
//!
//! # Preprocessing Types
//!
//! - [`ImageStats`] - Normalization statistics (mean, std, aspect)
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - Training pipelines
//! - Inference servers
//! - Dataset tools
//! - Validation scripts
//!
//! # Design Philosophy
//!
//! These are **ML inference and dataset types**. Raw sensor data belongs in
//! `sensor-types`. This separation enables:
//! - Clean boundaries between sensing and perception
//! - Sim ↔ Real parity with shared type definitions
//! - TFX-style dataset pipelines
//!
//! # Example
//!
//! ```
//! use ml_types::{Frame, DetectionResult, BoundingBox};
//!
//! // Create a frame for inference
//! let frame = Frame {
//!     id: 1,
//!     timestamp: 0.033,
//!     rgba: Some(vec![0u8; 640 * 480 * 4]),
//!     size: (640, 480),
//!     path: None,
//! };
//!
//! // Process detection results
//! let result = DetectionResult {
//!     frame_id: 1,
//!     positive: true,
//!     confidence: 0.95,
//!     boxes: vec![BoundingBox::new(0.1, 0.2, 0.3, 0.4, 0.95, 0)],
//! };
//!
//! assert!(result.positive);
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - ≥90% test coverage
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

mod bbox;
mod detection;
mod error;
mod frame;
mod keypoints;
mod label;
mod manifest;
mod metadata;
mod segmentation;
mod stats;
mod tracking;
mod validation;

// Re-export inference types
pub use bbox::BoundingBox;
pub use detection::DetectionResult;
pub use frame::Frame;
pub use keypoints::{Keypoint, KeypointSet, Skeleton};
pub use segmentation::{InstanceMask, SegmentationMask};
pub use tracking::{TrackId, TrackLifecycle, TrackRegistry, TrackingState};

// Re-export dataset schema types
pub use label::{ClassLabel, DetectionLabel, LabelSource};
pub use manifest::{DatasetManifest, RunManifest, SchemaVersion};
pub use metadata::{CaptureMetadata, FrameMetadata};

// Re-export preprocessing types
pub use stats::ImageStats;

// Re-export validation
pub use validation::ValidationError;

// Re-export error types
pub use error::{MlTypesError, Result};
