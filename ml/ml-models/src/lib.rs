//! Burn ML model architectures and checkpoint persistence for CortenForge.
//!
//! This crate provides neural network architectures built with the Burn framework,
//! along with checkpoint save/load functionality.
//!
//! # Model Architectures
//!
//! - [`LinearClassifier`] - Simple feedforward network for binary classification
//! - [`MultiboxModel`] - Multi-box detection model with spatial output heads
//!
//! # Checkpoint Persistence
//!
//! Models can save and load their weights using Burn's recorder system:
//! - Binary format (compact, fast)
//! - JSON format (human-readable, debuggable)
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//! - Training pipelines
//! - Inference servers
//! - CLI tools
//! - Embedded systems
//!
//! # Backend Support
//!
//! Models are generic over Burn backends. Common choices:
//! - `burn-ndarray` - CPU inference/training (default)
//! - `burn-wgpu` - GPU inference/training (optional feature)
//!
//! # Example
//!
//! ```ignore
//! use burn::tensor::backend::Backend;
//! use ml_models::{LinearClassifier, LinearClassifierConfig};
//!
//! // Create a classifier
//! let config = LinearClassifierConfig::default();
//! let device = Default::default();
//! let model = LinearClassifier::<MyBackend>::new(config, &device);
//!
//! // Run inference
//! let input = Tensor::zeros([1, 4], &device);
//! let output = model.forward(input);
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

mod backend;
mod checkpoint;
mod classifier;
mod error;
mod multibox;

// Re-export model types
pub use classifier::{LinearClassifier, LinearClassifierConfig};
pub use multibox::{MultiboxModel, MultiboxModelConfig};

// Re-export checkpoint utilities
pub use checkpoint::{CheckpointFormat, load_checkpoint, save_checkpoint};

// Re-export backend utilities
pub use backend::BackendType;

// Re-export error types
pub use error::{ModelError, Result};

/// Prelude for convenient imports.
pub mod prelude {
    pub use super::{
        BackendType, CheckpointFormat, LinearClassifier, LinearClassifierConfig, ModelError,
        MultiboxModel, MultiboxModelConfig, load_checkpoint, save_checkpoint,
    };
}
