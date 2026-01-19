//! Training loop implementation.

use serde::{Deserialize, Serialize};

use crate::config::TrainingConfig;
use crate::metrics::{EpochMetrics, TrainingMetrics};

/// State of a training run.
///
/// # Example
///
/// ```
/// use ml_training::TrainingState;
///
/// let state = TrainingState::new();
/// assert_eq!(state.epoch, 0);
/// assert!(!state.is_finished());
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrainingState {
    /// Current epoch (0-indexed).
    pub epoch: usize,

    /// Current batch within epoch.
    pub batch: usize,

    /// Total epochs to run.
    pub total_epochs: usize,

    /// Best validation loss seen.
    pub best_val_loss: Option<f32>,

    /// Epochs without improvement (for early stopping).
    pub epochs_without_improvement: usize,

    /// Whether training has finished.
    pub finished: bool,

    /// Accumulated metrics.
    pub metrics: TrainingMetrics,
}

impl Default for TrainingState {
    fn default() -> Self {
        Self::new()
    }
}

impl TrainingState {
    /// Creates a new training state.
    #[must_use]
    pub fn new() -> Self {
        Self {
            epoch: 0,
            batch: 0,
            total_epochs: 0,
            best_val_loss: None,
            epochs_without_improvement: 0,
            finished: false,
            metrics: TrainingMetrics::new(),
        }
    }

    /// Creates a training state for the given config.
    #[must_use]
    pub fn from_config(config: &TrainingConfig) -> Self {
        Self {
            epoch: 0,
            batch: 0,
            total_epochs: config.epochs,
            best_val_loss: None,
            epochs_without_improvement: 0,
            finished: false,
            metrics: TrainingMetrics::new(),
        }
    }

    /// Returns true if training is finished.
    #[must_use]
    pub const fn is_finished(&self) -> bool {
        self.finished
    }

    /// Returns the progress as a fraction [0, 1].
    #[must_use]
    #[allow(clippy::cast_precision_loss)]
    pub fn progress(&self) -> f32 {
        if self.total_epochs == 0 {
            0.0
        } else {
            self.epoch as f32 / self.total_epochs as f32
        }
    }

    /// Advances to the next epoch.
    pub const fn next_epoch(&mut self) {
        self.epoch += 1;
        self.batch = 0;
        if self.epoch >= self.total_epochs {
            self.finished = true;
        }
    }

    /// Advances to the next batch.
    pub const fn next_batch(&mut self) {
        self.batch += 1;
    }

    /// Records validation loss and checks for improvement.
    ///
    /// Returns true if this is a new best.
    pub fn record_val_loss(&mut self, val_loss: f32) -> bool {
        let improved =
            self.best_val_loss.is_none() || val_loss < self.best_val_loss.unwrap_or(f32::MAX);

        if improved {
            self.best_val_loss = Some(val_loss);
            self.epochs_without_improvement = 0;
        } else {
            self.epochs_without_improvement += 1;
        }

        improved
    }

    /// Checks if early stopping should trigger.
    #[must_use]
    pub const fn should_early_stop(&self, patience: usize) -> bool {
        patience > 0 && self.epochs_without_improvement >= patience
    }

    /// Marks training as early stopped.
    pub fn early_stop(&mut self, reason: impl Into<String>) {
        self.finished = true;
        self.metrics.set_early_stopped(reason);
    }

    /// Adds epoch metrics.
    pub fn add_epoch_metrics(&mut self, metrics: EpochMetrics) {
        self.metrics.add_epoch(metrics);
    }
}

/// Trainer for running training loops.
///
/// This provides the training loop structure. Actual training
/// requires passing in model-specific closures.
///
/// # Example
///
/// ```
/// use ml_training::{Trainer, TrainingConfig};
///
/// let config = TrainingConfig::new(10);
/// let trainer = Trainer::new(config);
///
/// assert_eq!(trainer.config().epochs, 10);
/// ```
#[derive(Debug, Clone)]
pub struct Trainer {
    config: TrainingConfig,
}

impl Default for Trainer {
    fn default() -> Self {
        Self::new(TrainingConfig::default())
    }
}

impl Trainer {
    /// Creates a new trainer with the given config.
    #[must_use]
    pub const fn new(config: TrainingConfig) -> Self {
        Self { config }
    }

    /// Returns the training configuration.
    #[must_use]
    pub const fn config(&self) -> &TrainingConfig {
        &self.config
    }

    /// Creates initial training state.
    #[must_use]
    pub fn initial_state(&self) -> TrainingState {
        TrainingState::from_config(&self.config)
    }

    /// Computes the learning rate for the current epoch.
    #[must_use]
    pub fn compute_lr(&self, epoch: usize) -> f32 {
        self.config.lr_schedule.compute_lr(
            self.config.optimizer.learning_rate,
            epoch,
            self.config.epochs,
        )
    }

    /// Returns whether validation should run this epoch.
    #[must_use]
    pub const fn should_validate(&self, epoch: usize) -> bool {
        self.config.val_frequency > 0 && (epoch + 1) % self.config.val_frequency == 0
    }

    /// Returns whether a checkpoint should be saved this epoch.
    #[must_use]
    pub const fn should_checkpoint(&self, epoch: usize) -> bool {
        self.config.checkpoint_frequency > 0 && (epoch + 1) % self.config.checkpoint_frequency == 0
    }

    /// Computes the number of batches for a dataset size.
    #[must_use]
    pub const fn num_batches(&self, dataset_size: usize) -> usize {
        if self.config.batch_size == 0 {
            0
        } else {
            dataset_size.div_ceil(self.config.batch_size)
        }
    }

    /// Gets batch indices for a given batch number.
    ///
    /// Returns (start, end) indices into the dataset.
    #[must_use]
    pub fn batch_indices(&self, batch: usize, dataset_size: usize) -> (usize, usize) {
        let start = batch * self.config.batch_size;
        let end = ((batch + 1) * self.config.batch_size).min(dataset_size);
        (start, end)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn training_state_new() {
        let state = TrainingState::new();
        assert_eq!(state.epoch, 0);
        assert_eq!(state.batch, 0);
        assert!(!state.is_finished());
        assert!(state.best_val_loss.is_none());
    }

    #[test]
    fn training_state_from_config() {
        let config = TrainingConfig::new(50);
        let state = TrainingState::from_config(&config);
        assert_eq!(state.total_epochs, 50);
    }

    #[test]
    fn training_state_progress() {
        let mut state = TrainingState::new();
        state.total_epochs = 10;

        assert!(state.progress().abs() < 1e-6);
        state.epoch = 5;
        assert!((state.progress() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn training_state_next_epoch() {
        let mut state = TrainingState::new();
        state.total_epochs = 2;
        state.batch = 10;

        state.next_epoch();
        assert_eq!(state.epoch, 1);
        assert_eq!(state.batch, 0);
        assert!(!state.is_finished());

        state.next_epoch();
        assert_eq!(state.epoch, 2);
        assert!(state.is_finished());
    }

    #[test]
    fn training_state_next_batch() {
        let mut state = TrainingState::new();
        state.next_batch();
        state.next_batch();
        assert_eq!(state.batch, 2);
    }

    #[test]
    fn training_state_record_val_loss_improvement() {
        let mut state = TrainingState::new();

        assert!(state.record_val_loss(0.5)); // First is best
        assert_eq!(state.best_val_loss, Some(0.5));
        assert_eq!(state.epochs_without_improvement, 0);

        assert!(state.record_val_loss(0.3)); // New best
        assert_eq!(state.best_val_loss, Some(0.3));
        assert_eq!(state.epochs_without_improvement, 0);
    }

    #[test]
    fn training_state_record_val_loss_no_improvement() {
        let mut state = TrainingState::new();

        state.record_val_loss(0.3);
        assert!(!state.record_val_loss(0.4)); // No improvement
        assert_eq!(state.epochs_without_improvement, 1);

        assert!(!state.record_val_loss(0.5)); // Still no improvement
        assert_eq!(state.epochs_without_improvement, 2);
    }

    #[test]
    fn training_state_early_stop_check() {
        let mut state = TrainingState::new();
        state.epochs_without_improvement = 5;

        assert!(!state.should_early_stop(10)); // Not yet
        assert!(state.should_early_stop(5)); // Exactly at patience
        assert!(state.should_early_stop(3)); // Past patience
        assert!(!state.should_early_stop(0)); // Disabled
    }

    #[test]
    fn training_state_early_stop() {
        let mut state = TrainingState::new();
        state.early_stop("no improvement");

        assert!(state.is_finished());
        assert!(state.metrics.early_stopped);
    }

    #[test]
    fn trainer_new() {
        let config = TrainingConfig::new(100);
        let trainer = Trainer::new(config);
        assert_eq!(trainer.config().epochs, 100);
    }

    #[test]
    fn trainer_default() {
        let trainer = Trainer::default();
        assert_eq!(trainer.config().epochs, 100);
    }

    #[test]
    fn trainer_compute_lr() {
        let config =
            TrainingConfig::new(100).with_optimizer(crate::config::OptimizerConfig::adam(1e-3));
        let trainer = Trainer::new(config);

        let lr = trainer.compute_lr(0);
        assert!((lr - 1e-3).abs() < 1e-6);
    }

    #[test]
    fn trainer_should_validate() {
        let config = TrainingConfig::new(100);
        let trainer = Trainer::new(config);

        // val_frequency = 1 by default
        assert!(trainer.should_validate(0)); // After epoch 0 (epoch 1 completed)
        assert!(trainer.should_validate(1));
        assert!(trainer.should_validate(99));
    }

    #[test]
    fn trainer_should_checkpoint() {
        let mut config = TrainingConfig::new(100);
        config.checkpoint_frequency = 10;
        let trainer = Trainer::new(config);

        assert!(!trainer.should_checkpoint(0)); // Epoch 1 - no
        assert!(trainer.should_checkpoint(9)); // Epoch 10 - yes
        assert!(!trainer.should_checkpoint(10)); // Epoch 11 - no
        assert!(trainer.should_checkpoint(19)); // Epoch 20 - yes
    }

    #[test]
    fn trainer_num_batches() {
        let config = TrainingConfig::new(10).with_batch_size(32);
        let trainer = Trainer::new(config);

        assert_eq!(trainer.num_batches(100), 4); // ceil(100/32) = 4
        assert_eq!(trainer.num_batches(32), 1);
        assert_eq!(trainer.num_batches(33), 2);
        assert_eq!(trainer.num_batches(0), 0);
    }

    #[test]
    fn trainer_batch_indices() {
        let config = TrainingConfig::new(10).with_batch_size(32);
        let trainer = Trainer::new(config);

        assert_eq!(trainer.batch_indices(0, 100), (0, 32));
        assert_eq!(trainer.batch_indices(1, 100), (32, 64));
        assert_eq!(trainer.batch_indices(2, 100), (64, 96));
        assert_eq!(trainer.batch_indices(3, 100), (96, 100)); // Last batch is partial
    }

    #[test]
    fn training_state_serialization() {
        let state = TrainingState::new();
        let json = serde_json::to_string(&state);
        assert!(json.is_ok());

        let parsed: std::result::Result<TrainingState, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), state);
    }
}
