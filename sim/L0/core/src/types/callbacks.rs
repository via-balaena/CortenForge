//! Callback types for user-defined hooks in the physics pipeline (DT-79).
//!
//! MuJoCo uses global function pointers (`mjcb_passive`, `mjcb_control`, etc.).
//! We use per-Model, thread-safe callback hooks:
//!
//! - `Arc<dyn Fn>` preserves `#[derive(Clone)]` on Model (Arc is Clone)
//! - `Fn` (not `FnMut`) is thread-safe with immutable captures
//! - `Send + Sync` bounds enable cross-thread sharing (e.g., BatchSim)
//! - `Option<Callback<...>>` — None = no overhead (branch predicted away)

use std::fmt;
use std::sync::Arc;

use super::data::Data;
use super::model::Model;

/// Thread-safe callback wrapper that implements Debug.
///
/// Wraps `Arc<dyn Fn(...) + Send + Sync>` and provides a Debug impl
/// (since `dyn Fn` doesn't implement Debug).
pub struct Callback<F: ?Sized>(pub Arc<F>);

impl<F: ?Sized> Clone for Callback<F> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<F: ?Sized> fmt::Debug for Callback<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("Callback(<fn>)")
    }
}

// ==================== Callback Type Aliases ====================

/// Passive force callback: called at end of `mj_fwd_passive()`.
///
/// Use to inject custom passive forces (e.g., viscous drag, spring models).
/// The callback may modify `data.qfrc_passive` or any other Data field.
pub type CbPassive = Callback<dyn Fn(&Model, &mut Data) + Send + Sync>;

/// Control callback: called at start of `mj_fwd_actuation()`.
///
/// Use to set `data.ctrl` from a controller (e.g., RL policy output).
pub type CbControl = Callback<dyn Fn(&Model, &mut Data) + Send + Sync>;

/// Contact filter callback: called after affinity check in collision.
///
/// Arguments: `(model, data, geom1_id, geom2_id)`.
/// Return `true` to KEEP the contact, `false` to REJECT it.
pub type CbContactFilter = Callback<dyn Fn(&Model, &Data, usize, usize) -> bool + Send + Sync>;
