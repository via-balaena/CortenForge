//! Training metrics and logging.

use serde::{Deserialize, Serialize};

/// Metrics for a single training epoch.
///
/// # Example
///
/// ```
/// use ml_training::EpochMetrics;
///
/// let metrics = EpochMetrics::new(0, 0.5, Some(0.4));
/// assert_eq!(metrics.epoch, 0);
/// assert!((metrics.train_loss - 0.5).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EpochMetrics {
    /// Epoch number (0-indexed).
    pub epoch: usize,

    /// Training loss for this epoch.
    pub train_loss: f32,

    /// Validation loss (if computed).
    pub val_loss: Option<f32>,

    /// Learning rate used.
    pub learning_rate: f32,

    /// Training time in seconds.
    pub train_time_secs: f32,

    /// Validation time in seconds.
    pub val_time_secs: Option<f32>,

    /// Number of training samples processed.
    pub train_samples: usize,

    /// Number of validation samples processed.
    pub val_samples: Option<usize>,

    /// Optional additional metrics (mAP, precision, recall, etc.).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub extra_metrics: Vec<(String, f32)>,
}

impl EpochMetrics {
    /// Creates new epoch metrics.
    #[must_use]
    pub const fn new(epoch: usize, train_loss: f32, val_loss: Option<f32>) -> Self {
        Self {
            epoch,
            train_loss,
            val_loss,
            learning_rate: 0.0,
            train_time_secs: 0.0,
            val_time_secs: None,
            train_samples: 0,
            val_samples: None,
            extra_metrics: Vec::new(), // const-allowed in const fn
        }
    }

    /// Sets the learning rate.
    #[must_use]
    pub const fn with_learning_rate(mut self, lr: f32) -> Self {
        self.learning_rate = lr;
        self
    }

    /// Sets the training time.
    #[must_use]
    pub const fn with_train_time(mut self, secs: f32) -> Self {
        self.train_time_secs = secs;
        self
    }

    /// Sets the validation time.
    #[must_use]
    pub const fn with_val_time(mut self, secs: f32) -> Self {
        self.val_time_secs = Some(secs);
        self
    }

    /// Sets sample counts.
    #[must_use]
    pub const fn with_samples(mut self, train: usize, val: Option<usize>) -> Self {
        self.train_samples = train;
        self.val_samples = val;
        self
    }

    /// Adds an extra metric.
    #[must_use]
    pub fn with_metric(mut self, name: impl Into<String>, value: f32) -> Self {
        self.extra_metrics.push((name.into(), value));
        self
    }

    /// Returns total time (train + val) in seconds.
    #[must_use]
    pub fn total_time_secs(&self) -> f32 {
        self.train_time_secs + self.val_time_secs.unwrap_or(0.0)
    }

    /// Gets an extra metric by name.
    #[must_use]
    pub fn get_metric(&self, name: &str) -> Option<f32> {
        self.extra_metrics
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| *v)
    }

    /// Returns true if validation loss improved (is lower than previous best).
    #[must_use]
    pub fn val_improved(&self, previous_best: Option<f32>) -> bool {
        match (self.val_loss, previous_best) {
            (Some(current), Some(best)) => current < best,
            (Some(_), None) => true, // First validation
            (None, _) => false,
        }
    }
}

/// Aggregate metrics for a training run.
///
/// # Example
///
/// ```
/// use ml_training::{TrainingMetrics, EpochMetrics};
///
/// let mut metrics = TrainingMetrics::new();
/// metrics.add_epoch(EpochMetrics::new(0, 0.5, Some(0.4)));
/// metrics.add_epoch(EpochMetrics::new(1, 0.3, Some(0.35)));
///
/// assert_eq!(metrics.epochs_completed(), 2);
/// assert!((metrics.final_loss() - 0.3).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TrainingMetrics {
    /// Metrics for each epoch.
    pub epoch_metrics: Vec<EpochMetrics>,

    /// Best validation loss seen.
    pub best_val_loss: Option<f32>,

    /// Epoch with best validation loss.
    pub best_epoch: Option<usize>,

    /// Total training time in seconds.
    pub total_time_secs: f32,

    /// Whether training was early stopped.
    pub early_stopped: bool,

    /// Reason for stopping (if not completed normally).
    pub stop_reason: Option<String>,
}

impl TrainingMetrics {
    /// Creates new empty training metrics.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds metrics for an epoch.
    pub fn add_epoch(&mut self, metrics: EpochMetrics) {
        // Update best validation loss
        if let Some(val_loss) = metrics.val_loss {
            if self.best_val_loss.is_none() || val_loss < self.best_val_loss.unwrap_or(f32::MAX) {
                self.best_val_loss = Some(val_loss);
                self.best_epoch = Some(metrics.epoch);
            }
        }

        self.total_time_secs += metrics.total_time_secs();
        self.epoch_metrics.push(metrics);
    }

    /// Returns the number of completed epochs.
    #[must_use]
    pub fn epochs_completed(&self) -> usize {
        self.epoch_metrics.len()
    }

    /// Returns the final training loss.
    #[must_use]
    pub fn final_loss(&self) -> f32 {
        self.epoch_metrics.last().map_or(f32::NAN, |m| m.train_loss)
    }

    /// Returns the final validation loss.
    #[must_use]
    pub fn final_val_loss(&self) -> Option<f32> {
        self.epoch_metrics.last().and_then(|m| m.val_loss)
    }

    /// Returns the initial training loss.
    #[must_use]
    pub fn initial_loss(&self) -> f32 {
        self.epoch_metrics
            .first()
            .map_or(f32::NAN, |m| m.train_loss)
    }

    /// Returns the loss improvement ratio.
    #[must_use]
    pub fn loss_improvement(&self) -> f32 {
        let initial = self.initial_loss();
        let final_loss = self.final_loss();
        if initial > 0.0 && !initial.is_nan() && !final_loss.is_nan() {
            1.0 - (final_loss / initial)
        } else {
            0.0
        }
    }

    /// Returns training losses as a vector.
    #[must_use]
    pub fn train_losses(&self) -> Vec<f32> {
        self.epoch_metrics.iter().map(|m| m.train_loss).collect()
    }

    /// Returns validation losses as a vector.
    #[must_use]
    pub fn val_losses(&self) -> Vec<Option<f32>> {
        self.epoch_metrics.iter().map(|m| m.val_loss).collect()
    }

    /// Returns learning rates as a vector.
    #[must_use]
    pub fn learning_rates(&self) -> Vec<f32> {
        self.epoch_metrics.iter().map(|m| m.learning_rate).collect()
    }

    /// Marks training as early stopped.
    pub fn set_early_stopped(&mut self, reason: impl Into<String>) {
        self.early_stopped = true;
        self.stop_reason = Some(reason.into());
    }

    /// Returns a human-readable summary.
    #[must_use]
    #[allow(clippy::let_underscore_must_use)] // String::write_fmt is infallible
    pub fn summary(&self) -> String {
        use std::fmt::Write;

        let mut s = String::new();
        let _ = writeln!(s, "Training Summary");
        let _ = writeln!(s, "================");
        let _ = writeln!(s, "Epochs completed: {}", self.epochs_completed());
        let _ = writeln!(s, "Total time: {:.1}s", self.total_time_secs);
        let _ = writeln!(
            s,
            "Initial loss: {:.4} -> Final loss: {:.4}",
            self.initial_loss(),
            self.final_loss()
        );
        let _ = writeln!(s, "Improvement: {:.1}%", self.loss_improvement() * 100.0);

        if let Some(best) = self.best_val_loss {
            let _ = writeln!(
                s,
                "Best val loss: {:.4} (epoch {})",
                best,
                self.best_epoch.unwrap_or(0)
            );
        }

        if self.early_stopped {
            let _ = writeln!(
                s,
                "Early stopped: {}",
                self.stop_reason.as_deref().unwrap_or("yes")
            );
        }

        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epoch_metrics_new() {
        let metrics = EpochMetrics::new(0, 0.5, Some(0.4));
        assert_eq!(metrics.epoch, 0);
        assert!((metrics.train_loss - 0.5).abs() < 1e-6);
        assert!((metrics.val_loss.unwrap() - 0.4).abs() < 1e-6);
    }

    #[test]
    fn epoch_metrics_builder() {
        let metrics = EpochMetrics::new(1, 0.3, None)
            .with_learning_rate(1e-3)
            .with_train_time(10.0)
            .with_val_time(2.0)
            .with_samples(1000, Some(200))
            .with_metric("mAP", 0.85);

        assert!((metrics.learning_rate - 1e-3).abs() < 1e-6);
        assert!((metrics.train_time_secs - 10.0).abs() < 1e-6);
        assert!((metrics.val_time_secs.unwrap() - 2.0).abs() < 1e-6);
        assert_eq!(metrics.train_samples, 1000);
        assert_eq!(metrics.val_samples, Some(200));
        assert!((metrics.get_metric("mAP").unwrap() - 0.85).abs() < 1e-6);
    }

    #[test]
    fn epoch_metrics_total_time() {
        let metrics = EpochMetrics::new(0, 0.5, None)
            .with_train_time(10.0)
            .with_val_time(2.0);

        assert!((metrics.total_time_secs() - 12.0).abs() < 1e-6);
    }

    #[test]
    fn epoch_metrics_val_improved() {
        let metrics = EpochMetrics::new(0, 0.5, Some(0.4));

        assert!(metrics.val_improved(Some(0.5))); // 0.4 < 0.5
        assert!(!metrics.val_improved(Some(0.3))); // 0.4 > 0.3
        assert!(metrics.val_improved(None)); // First validation
    }

    #[test]
    fn epoch_metrics_val_improved_no_val() {
        let metrics = EpochMetrics::new(0, 0.5, None);
        assert!(!metrics.val_improved(Some(0.5)));
    }

    #[test]
    fn training_metrics_new() {
        let metrics = TrainingMetrics::new();
        assert!(metrics.epoch_metrics.is_empty());
        assert_eq!(metrics.epochs_completed(), 0);
    }

    #[test]
    fn training_metrics_add_epoch() {
        let mut metrics = TrainingMetrics::new();
        metrics.add_epoch(EpochMetrics::new(0, 0.5, Some(0.4)).with_train_time(10.0));
        metrics.add_epoch(EpochMetrics::new(1, 0.3, Some(0.35)).with_train_time(10.0));

        assert_eq!(metrics.epochs_completed(), 2);
        assert!((metrics.final_loss() - 0.3).abs() < 1e-6);
        assert!((metrics.final_val_loss().unwrap() - 0.35).abs() < 1e-6);
        assert!((metrics.best_val_loss.unwrap() - 0.35).abs() < 1e-6);
        assert_eq!(metrics.best_epoch, Some(1));
        assert!((metrics.total_time_secs - 20.0).abs() < 1e-6);
    }

    #[test]
    fn training_metrics_initial_loss() {
        let mut metrics = TrainingMetrics::new();
        metrics.add_epoch(EpochMetrics::new(0, 1.0, None));
        metrics.add_epoch(EpochMetrics::new(1, 0.5, None));

        assert!((metrics.initial_loss() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn training_metrics_loss_improvement() {
        let mut metrics = TrainingMetrics::new();
        metrics.add_epoch(EpochMetrics::new(0, 1.0, None));
        metrics.add_epoch(EpochMetrics::new(1, 0.5, None));

        assert!((metrics.loss_improvement() - 0.5).abs() < 1e-6); // 50% improvement
    }

    #[test]
    fn training_metrics_losses() {
        let mut metrics = TrainingMetrics::new();
        metrics.add_epoch(EpochMetrics::new(0, 0.5, Some(0.4)));
        metrics.add_epoch(EpochMetrics::new(1, 0.3, Some(0.35)));

        let train_losses = metrics.train_losses();
        assert_eq!(train_losses.len(), 2);
        assert!((train_losses[0] - 0.5).abs() < 1e-6);
        assert!((train_losses[1] - 0.3).abs() < 1e-6);

        let val_losses = metrics.val_losses();
        assert_eq!(val_losses.len(), 2);
        assert!((val_losses[0].unwrap() - 0.4).abs() < 1e-6);
    }

    #[test]
    fn training_metrics_early_stopped() {
        let mut metrics = TrainingMetrics::new();
        metrics.set_early_stopped("no improvement for 10 epochs");

        assert!(metrics.early_stopped);
        assert_eq!(
            metrics.stop_reason,
            Some("no improvement for 10 epochs".to_string())
        );
    }

    #[test]
    fn training_metrics_summary() {
        let mut metrics = TrainingMetrics::new();
        metrics.add_epoch(EpochMetrics::new(0, 1.0, Some(0.9)).with_train_time(5.0));
        metrics.add_epoch(EpochMetrics::new(1, 0.5, Some(0.45)).with_train_time(5.0));

        let summary = metrics.summary();
        assert!(summary.contains("Epochs completed: 2"));
        assert!(summary.contains("Total time:"));
        assert!(summary.contains("Best val loss:"));
    }

    #[test]
    fn epoch_metrics_serialization() {
        let metrics = EpochMetrics::new(0, 0.5, Some(0.4)).with_metric("mAP", 0.85);

        let json = serde_json::to_string(&metrics);
        assert!(json.is_ok());

        let parsed: std::result::Result<EpochMetrics, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        if let Ok(m) = parsed {
            assert_eq!(m, metrics);
        }
    }

    #[test]
    fn training_metrics_serialization() {
        let mut metrics = TrainingMetrics::new();
        metrics.add_epoch(EpochMetrics::new(0, 0.5, Some(0.4)));

        let json = serde_json::to_string(&metrics);
        assert!(json.is_ok());

        let parsed: std::result::Result<TrainingMetrics, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), metrics);
    }
}
