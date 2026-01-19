//! Training configuration.

use serde::{Deserialize, Serialize};

/// Configuration for a training run.
///
/// # Example
///
/// ```
/// use ml_training::TrainingConfig;
///
/// let config = TrainingConfig::default();
/// assert_eq!(config.epochs, 100);
/// assert_eq!(config.batch_size, 32);
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrainingConfig {
    /// Number of training epochs.
    pub epochs: usize,

    /// Batch size.
    pub batch_size: usize,

    /// Optimizer configuration.
    pub optimizer: OptimizerConfig,

    /// Learning rate schedule.
    pub lr_schedule: LearningRateSchedule,

    /// Whether to shuffle data each epoch.
    pub shuffle: bool,

    /// Validation frequency (epochs between validations).
    pub val_frequency: usize,

    /// Checkpoint frequency (epochs between saves).
    pub checkpoint_frequency: usize,

    /// Early stopping patience (0 = disabled).
    pub early_stopping_patience: usize,

    /// Gradient clipping threshold (0.0 = disabled).
    pub gradient_clip: f32,

    /// Random seed for reproducibility.
    pub seed: Option<u64>,
}

impl Default for TrainingConfig {
    fn default() -> Self {
        Self {
            epochs: 100,
            batch_size: 32,
            optimizer: OptimizerConfig::default(),
            lr_schedule: LearningRateSchedule::default(),
            shuffle: true,
            val_frequency: 1,
            checkpoint_frequency: 10,
            early_stopping_patience: 10,
            gradient_clip: 1.0,
            seed: None,
        }
    }
}

impl TrainingConfig {
    /// Creates a new training config with the given epochs.
    #[must_use]
    pub const fn new(epochs: usize) -> Self {
        Self {
            epochs,
            batch_size: 32,
            optimizer: OptimizerConfig::adam(1e-3),
            lr_schedule: LearningRateSchedule::Constant,
            shuffle: true,
            val_frequency: 1,
            checkpoint_frequency: 10,
            early_stopping_patience: 10,
            gradient_clip: 1.0,
            seed: None,
        }
    }

    /// Sets the batch size.
    #[must_use]
    pub const fn with_batch_size(mut self, batch_size: usize) -> Self {
        self.batch_size = batch_size;
        self
    }

    /// Sets the optimizer.
    #[must_use]
    pub const fn with_optimizer(mut self, optimizer: OptimizerConfig) -> Self {
        self.optimizer = optimizer;
        self
    }

    /// Sets the learning rate schedule.
    #[must_use]
    pub const fn with_lr_schedule(mut self, schedule: LearningRateSchedule) -> Self {
        self.lr_schedule = schedule;
        self
    }

    /// Sets the random seed.
    #[must_use]
    pub const fn with_seed(mut self, seed: u64) -> Self {
        self.seed = Some(seed);
        self
    }

    /// Disables shuffling.
    #[must_use]
    pub const fn without_shuffle(mut self) -> Self {
        self.shuffle = false;
        self
    }

    /// Validates the configuration.
    ///
    /// Returns `true` if all values are valid.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.epochs > 0
            && self.batch_size > 0
            && self.val_frequency > 0
            && self.checkpoint_frequency > 0
            && self.gradient_clip >= 0.0
            && self.optimizer.is_valid()
    }
}

/// Optimizer configuration.
///
/// # Example
///
/// ```
/// use ml_training::OptimizerConfig;
///
/// let adam = OptimizerConfig::adam(1e-3);
/// assert_eq!(adam.learning_rate, 1e-3);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OptimizerConfig {
    /// Base learning rate.
    pub learning_rate: f32,

    /// Weight decay (L2 regularization).
    pub weight_decay: f32,

    /// Optimizer type.
    pub optimizer_type: OptimizerType,

    /// Momentum (for SGD).
    pub momentum: f32,

    /// Beta1 (for Adam).
    pub beta1: f32,

    /// Beta2 (for Adam).
    pub beta2: f32,

    /// Epsilon for numerical stability.
    pub epsilon: f32,
}

impl Default for OptimizerConfig {
    fn default() -> Self {
        Self::adam(1e-3)
    }
}

impl OptimizerConfig {
    /// Creates an Adam optimizer config.
    #[must_use]
    pub const fn adam(learning_rate: f32) -> Self {
        Self {
            learning_rate,
            weight_decay: 0.0,
            optimizer_type: OptimizerType::Adam,
            momentum: 0.0,
            beta1: 0.9,
            beta2: 0.999,
            epsilon: 1e-8,
        }
    }

    /// Creates an SGD optimizer config.
    #[must_use]
    pub const fn sgd(learning_rate: f32) -> Self {
        Self {
            learning_rate,
            weight_decay: 0.0,
            optimizer_type: OptimizerType::Sgd,
            momentum: 0.0,
            beta1: 0.0,
            beta2: 0.0,
            epsilon: 1e-8,
        }
    }

    /// Creates an SGD with momentum optimizer config.
    #[must_use]
    pub const fn sgd_momentum(learning_rate: f32, momentum: f32) -> Self {
        Self {
            learning_rate,
            weight_decay: 0.0,
            optimizer_type: OptimizerType::SgdMomentum,
            momentum,
            beta1: 0.0,
            beta2: 0.0,
            epsilon: 1e-8,
        }
    }

    /// Sets weight decay.
    #[must_use]
    pub const fn with_weight_decay(mut self, weight_decay: f32) -> Self {
        self.weight_decay = weight_decay;
        self
    }

    /// Validates the configuration.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.learning_rate > 0.0
            && self.weight_decay >= 0.0
            && self.momentum >= 0.0
            && self.momentum <= 1.0
            && self.beta1 >= 0.0
            && self.beta1 < 1.0
            && self.beta2 >= 0.0
            && self.beta2 < 1.0
            && self.epsilon > 0.0
    }
}

/// Type of optimizer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum OptimizerType {
    /// Stochastic Gradient Descent.
    Sgd,
    /// SGD with momentum.
    SgdMomentum,
    /// Adam optimizer.
    Adam,
    /// `AdamW` optimizer (Adam with decoupled weight decay).
    AdamW,
}

/// Learning rate schedule.
///
/// # Example
///
/// ```
/// use ml_training::LearningRateSchedule;
///
/// let schedule = LearningRateSchedule::step(0.1, 30);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub enum LearningRateSchedule {
    /// Constant learning rate.
    #[default]
    Constant,

    /// Step decay: multiply by factor every `step_size` epochs.
    Step {
        /// Decay factor.
        factor: f32,
        /// Epochs between decays.
        step_size: usize,
    },

    /// Exponential decay: lr * gamma^epoch.
    Exponential {
        /// Decay rate per epoch.
        gamma: f32,
    },

    /// Cosine annealing to minimum.
    Cosine {
        /// Minimum learning rate.
        min_lr: f32,
    },

    /// Linear warmup followed by cosine decay.
    WarmupCosine {
        /// Warmup epochs.
        warmup_epochs: usize,
        /// Minimum learning rate.
        min_lr: f32,
    },
}

impl LearningRateSchedule {
    /// Creates a step decay schedule.
    #[must_use]
    pub const fn step(factor: f32, step_size: usize) -> Self {
        Self::Step { factor, step_size }
    }

    /// Creates an exponential decay schedule.
    #[must_use]
    pub const fn exponential(gamma: f32) -> Self {
        Self::Exponential { gamma }
    }

    /// Creates a cosine annealing schedule.
    #[must_use]
    pub const fn cosine(min_lr: f32) -> Self {
        Self::Cosine { min_lr }
    }

    /// Creates a warmup + cosine schedule.
    #[must_use]
    pub const fn warmup_cosine(warmup_epochs: usize, min_lr: f32) -> Self {
        Self::WarmupCosine {
            warmup_epochs,
            min_lr,
        }
    }

    /// Computes the learning rate for a given epoch.
    ///
    /// # Arguments
    ///
    /// - `base_lr`: The base learning rate
    /// - `epoch`: Current epoch (0-indexed)
    /// - `total_epochs`: Total number of epochs
    #[must_use]
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_possible_wrap
    )]
    pub fn compute_lr(&self, base_lr: f32, epoch: usize, total_epochs: usize) -> f32 {
        match self {
            Self::Constant => base_lr,

            Self::Step { factor, step_size } => {
                let decays = epoch / step_size;
                base_lr * factor.powi(decays as i32)
            }

            Self::Exponential { gamma } => base_lr * gamma.powi(epoch as i32),

            Self::Cosine { min_lr } => {
                let progress = epoch as f32 / total_epochs as f32;
                let cosine = (std::f32::consts::PI * progress).cos();
                min_lr + (base_lr - min_lr) * (1.0 + cosine) / 2.0
            }

            Self::WarmupCosine {
                warmup_epochs,
                min_lr,
            } => {
                if epoch < *warmup_epochs {
                    // Linear warmup
                    base_lr * (epoch + 1) as f32 / *warmup_epochs as f32
                } else {
                    // Cosine decay
                    let remaining = total_epochs.saturating_sub(*warmup_epochs);
                    let progress = (epoch - warmup_epochs) as f32 / remaining.max(1) as f32;
                    let cosine = (std::f32::consts::PI * progress).cos();
                    min_lr + (base_lr - min_lr) * (1.0 + cosine) / 2.0
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn training_config_default() {
        let config = TrainingConfig::default();
        assert_eq!(config.epochs, 100);
        assert_eq!(config.batch_size, 32);
        assert!(config.shuffle);
        assert!(config.is_valid());
    }

    #[test]
    fn training_config_builder() {
        let config = TrainingConfig::new(50)
            .with_batch_size(64)
            .with_seed(42)
            .without_shuffle();

        assert_eq!(config.epochs, 50);
        assert_eq!(config.batch_size, 64);
        assert_eq!(config.seed, Some(42));
        assert!(!config.shuffle);
    }

    #[test]
    fn training_config_invalid() {
        let mut config = TrainingConfig::default();
        config.epochs = 0;
        assert!(!config.is_valid());

        config = TrainingConfig::default();
        config.batch_size = 0;
        assert!(!config.is_valid());
    }

    #[test]
    fn optimizer_config_adam() {
        let config = OptimizerConfig::adam(1e-3);
        assert_eq!(config.learning_rate, 1e-3);
        assert_eq!(config.optimizer_type, OptimizerType::Adam);
        assert!(config.is_valid());
    }

    #[test]
    fn optimizer_config_sgd() {
        let config = OptimizerConfig::sgd(0.01);
        assert_eq!(config.learning_rate, 0.01);
        assert_eq!(config.optimizer_type, OptimizerType::Sgd);
    }

    #[test]
    fn optimizer_config_sgd_momentum() {
        let config = OptimizerConfig::sgd_momentum(0.01, 0.9);
        assert_eq!(config.momentum, 0.9);
        assert_eq!(config.optimizer_type, OptimizerType::SgdMomentum);
    }

    #[test]
    fn optimizer_config_weight_decay() {
        let config = OptimizerConfig::adam(1e-3).with_weight_decay(1e-4);
        assert_eq!(config.weight_decay, 1e-4);
    }

    #[test]
    fn lr_schedule_constant() {
        let schedule = LearningRateSchedule::Constant;
        assert_eq!(schedule.compute_lr(0.01, 0, 100), 0.01);
        assert_eq!(schedule.compute_lr(0.01, 50, 100), 0.01);
        assert_eq!(schedule.compute_lr(0.01, 99, 100), 0.01);
    }

    #[test]
    fn lr_schedule_step() {
        let schedule = LearningRateSchedule::step(0.1, 30);

        // Before first step
        assert!((schedule.compute_lr(1.0, 0, 100) - 1.0).abs() < 1e-6);
        assert!((schedule.compute_lr(1.0, 29, 100) - 1.0).abs() < 1e-6);

        // After first step
        assert!((schedule.compute_lr(1.0, 30, 100) - 0.1).abs() < 1e-6);
        assert!((schedule.compute_lr(1.0, 59, 100) - 0.1).abs() < 1e-6);

        // After second step
        assert!((schedule.compute_lr(1.0, 60, 100) - 0.01).abs() < 1e-6);
    }

    #[test]
    fn lr_schedule_exponential() {
        let schedule = LearningRateSchedule::exponential(0.95);
        assert!((schedule.compute_lr(1.0, 0, 100) - 1.0).abs() < 1e-6);
        assert!((schedule.compute_lr(1.0, 1, 100) - 0.95).abs() < 1e-6);
        assert!((schedule.compute_lr(1.0, 2, 100) - 0.9025).abs() < 1e-5);
    }

    #[test]
    fn lr_schedule_cosine() {
        let schedule = LearningRateSchedule::cosine(0.0);

        // Start at full lr
        assert!((schedule.compute_lr(1.0, 0, 100) - 1.0).abs() < 1e-6);

        // Middle point
        let mid_lr = schedule.compute_lr(1.0, 50, 100);
        assert!((mid_lr - 0.5).abs() < 1e-5);

        // End at min_lr
        assert!((schedule.compute_lr(1.0, 100, 100) - 0.0).abs() < 1e-5);
    }

    #[test]
    fn lr_schedule_warmup_cosine() {
        let schedule = LearningRateSchedule::warmup_cosine(10, 0.0);

        // Warmup phase (linear increase)
        assert!((schedule.compute_lr(1.0, 0, 100) - 0.1).abs() < 1e-6);
        assert!((schedule.compute_lr(1.0, 4, 100) - 0.5).abs() < 1e-6);
        assert!((schedule.compute_lr(1.0, 9, 100) - 1.0).abs() < 1e-6);

        // After warmup, starts cosine decay
        let after_warmup = schedule.compute_lr(1.0, 10, 100);
        assert!(after_warmup <= 1.0);
    }

    #[test]
    fn config_serialization() {
        let config = TrainingConfig::default();
        let json = serde_json::to_string(&config);
        assert!(json.is_ok());

        let parsed: std::result::Result<TrainingConfig, _> =
            serde_json::from_str(&json.unwrap_or_default());
        assert!(parsed.is_ok());
        assert_eq!(parsed.unwrap_or_default(), config);
    }

    #[test]
    fn optimizer_config_serialization() {
        let config = OptimizerConfig::adam(1e-3);
        let json = serde_json::to_string(&config);
        assert!(json.is_ok());
    }

    #[test]
    fn lr_schedule_serialization() {
        let schedule = LearningRateSchedule::warmup_cosine(10, 1e-5);
        let json = serde_json::to_string(&schedule);
        assert!(json.is_ok());
    }
}
