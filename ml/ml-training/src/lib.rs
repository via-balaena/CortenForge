//! Training lifecycle for CortenForge ML models.
//!
//! This crate provides training utilities for ML models:
//!
//! # Training Components
//!
//! - [`TrainingConfig`] - Configuration for training runs
//! - [`Trainer`] - Training loop implementation
//! - [`TrainingMetrics`] - Metrics collected during training
//!
//! # Loss Functions
//!
//! - [`detection_loss`] - Combined box + classification loss
//! - [`box_loss`] - Bounding box regression loss (smooth L1)
//! - [`classification_loss`] - Binary cross-entropy for detection
//!
//! # Box Matching
//!
//! - [`BoxMatcher`] - Matches predictions to ground truth
//! - [`MatchResult`] - Result of box matching
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - Training pipelines
//! - Model evaluation
//! - Hyperparameter tuning
//!
//! # Example
//!
//! ```ignore
//! use ml_training::{TrainingConfig, Trainer};
//! use ml_models::LinearClassifier;
//!
//! let config = TrainingConfig::default();
//! let trainer = Trainer::new(config);
//!
//! // Train the model
//! let metrics = trainer.train(&mut model, &dataset)?;
//! println!("Final loss: {}", metrics.final_loss);
//! ```
//!
//! # Quality Standards
//!
//! This crate maintains A-grade standards per [STANDARDS.md](../../STANDARDS.md):
//! - Zero clippy/doc warnings
//! - Zero `unwrap`/`expect` in library code

// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]

mod config;
mod error;
mod loss;
mod matching;
mod metrics;
mod trainer;

// Re-export configuration
pub use config::{LearningRateSchedule, OptimizerConfig, TrainingConfig};

// Re-export loss functions
pub use loss::{
    LossWeights, box_loss, classification_loss, compute_iou, detection_loss, giou_loss,
};

// Re-export matching
pub use matching::{BoxMatcher, MatchResult, MatchStrategy};

// Re-export metrics
pub use metrics::{EpochMetrics, TrainingMetrics};

// Re-export trainer
pub use trainer::{Trainer, TrainingState};

// Re-export error types
pub use error::{Result, TrainingError};

/// Prelude for convenient imports.
pub mod prelude {
    pub use super::{
        BoxMatcher, EpochMetrics, LearningRateSchedule, LossWeights, MatchResult, MatchStrategy,
        OptimizerConfig, Trainer, TrainingConfig, TrainingError, TrainingMetrics, TrainingState,
        box_loss, classification_loss, compute_iou, detection_loss, giou_loss,
    };
}
