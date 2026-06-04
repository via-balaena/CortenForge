//! Progress reporting for the long-running steps (mold generation can
//! take many minutes). The core emits [`Progress`] updates through a
//! [`ProgressSink`]; a CLI prints them, a GUI drives a progress bar,
//! and tests collect them. Defined in the foundation so the step
//! implementations built on top have a stable channel to report into.

use serde::{Deserialize, Serialize};

/// A single progress update from a long-running operation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Progress {
    /// Completion in `0.0..=1.0`. Always clamped on construction.
    pub fraction: f32,
    /// Short stage label, e.g. `"Shaping layer 2 of 3"`.
    pub stage: String,
    /// Optional longer, reassuring detail for the user.
    pub detail: String,
}

impl Progress {
    /// Build a progress update, clamping `fraction` into `0.0..=1.0`
    /// so a sink never receives an out-of-range value.
    #[must_use]
    pub fn new(fraction: f32, stage: impl Into<String>, detail: impl Into<String>) -> Self {
        Self {
            fraction: fraction.clamp(0.0, 1.0),
            stage: stage.into(),
            detail: detail.into(),
        }
    }
}

/// A destination for [`Progress`] updates. Implemented for any
/// `FnMut(&Progress)` closure, so callers can pass a closure directly,
/// and by [`NoopSink`] for the "I don't care about progress" case.
pub trait ProgressSink {
    /// Receive one progress update.
    fn report(&mut self, progress: &Progress);
}

impl<F: FnMut(&Progress)> ProgressSink for F {
    fn report(&mut self, progress: &Progress) {
        self(progress);
    }
}

/// A [`ProgressSink`] that discards everything.
#[derive(Debug, Default, Clone, Copy)]
pub struct NoopSink;

impl ProgressSink for NoopSink {
    fn report(&mut self, _progress: &Progress) {}
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

    use super::*;

    #[test]
    fn fraction_is_clamped() {
        assert_eq!(Progress::new(-1.0, "s", "d").fraction, 0.0);
        assert_eq!(Progress::new(2.0, "s", "d").fraction, 1.0);
        assert_eq!(Progress::new(0.5, "s", "d").fraction, 0.5);
    }

    #[test]
    fn closure_is_a_sink() {
        let mut collected: Vec<f32> = Vec::new();
        {
            let mut sink = |p: &Progress| collected.push(p.fraction);
            sink.report(&Progress::new(0.25, "a", ""));
            sink.report(&Progress::new(0.75, "b", ""));
        }
        assert_eq!(collected, vec![0.25, 0.75]);
    }

    #[test]
    fn noop_sink_accepts_and_discards() {
        let mut sink = NoopSink;
        sink.report(&Progress::new(0.5, "ignored", "ignored"));
    }
}
