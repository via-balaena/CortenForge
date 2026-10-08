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

/// Passive force callback, the counterpart of MuJoCo's `mjcb_passive`.
///
/// Runs at the end of the passive-force computation in the velocity stage,
/// after `qfrc_spring`, `qfrc_damper`, `qfrc_gravcomp`, `qfrc_fluid` and their
/// sum `qfrc_passive` are written and before passive plugins. Add custom forces
/// to `qfrc_passive`. It runs before [`CbControl`] in the same pass: actuator
/// forces, `qfrc_bias`, constraint forces and `qacc` have not been computed for
/// this pass yet.
///
/// # When it runs (as MuJoCo 3.5.0)
///
/// - once per [`Data::forward`], [`Data::step1`], and [`Data::forward_skip`]
///   with [`MjStage::None`] or [`MjStage::Pos`];
/// - once per [`Data::step`] with Euler and the implicit integrators, four
///   times with RK4 (once per stage);
/// - never in [`Data::step2`] or `forward_skip(MjStage::Vel, _)`;
/// - never while both `DISABLE_SPRING` and `DISABLE_DAMPER` are set: passive
///   forces are skipped as a whole, callback and passive plugins included
///   (MuJoCo's `mj_passive` does the same), so a component that must always
///   act, a thermostat, goes silent under those two flags;
/// - also on a model with no degrees of freedom;
/// - inside finite-difference derivatives, many times per call.
///
/// Not matched: on the step that puts a tree to sleep, MuJoCo runs the forward
/// pass once more (both callbacks fire twice); sim-core does not.
///
/// [`MjStage::None`]: crate::MjStage::None
/// [`MjStage::Pos`]: crate::MjStage::Pos
pub type CbPassive = Callback<dyn Fn(&Model, &mut Data) + Send + Sync>;

/// Control callback, the counterpart of MuJoCo's `mjcb_control`.
///
/// Runs after the velocity stage (after [`CbPassive`]) and before actuation.
/// Set `ctrl` (or `qfrc_applied` / `xfrc_applied`) here.
///
/// # When it runs (as MuJoCo 3.5.0)
///
/// - once per [`Data::forward`] and per [`Data::forward_skip`] (every
///   [`MjStage`]), unless `DISABLE_ACTUATION` is set;
/// - in [`Data::step`]: once with Euler and the implicit integrators, four
///   times with RK4, each RK4 stage calling it at that stage's trial state and
///   time, so a state-dependent controller is re-evaluated (unless
///   `DISABLE_ACTUATION` is set);
/// - once per [`Data::step1`], even with `DISABLE_ACTUATION` set (MuJoCo's
///   `mj_step1` does not check the flag); never in [`Data::step2`];
/// - inside finite-difference derivatives.
///
/// [`MjStage`]: crate::MjStage
pub type CbControl = Callback<dyn Fn(&Model, &mut Data) + Send + Sync>;

/// Contact filter callback: called after affinity check in collision.
///
/// Arguments: `(model, data, geom1_id, geom2_id)`.
/// Return `true` to KEEP the contact, `false` to REJECT it.
///
/// **Polarity note**: MuJoCo's `mjcb_contactfilter` has the *opposite* convention —
/// it returns nonzero to REJECT. Our callback uses positive logic (true=keep) for
/// Rust idiom. Callers porting MuJoCo filters must invert the return value.
pub type CbContactFilter = Callback<dyn Fn(&Model, &Data, usize, usize) -> bool + Send + Sync>;

/// Sensor callback: called for `MjSensorType::User` sensors.
///
/// Arguments: `(model, data, sensor_id, stage)`.
/// The callback should write to `data.sensordata[adr..adr+dim]`.
pub type CbSensor =
    Callback<dyn Fn(&Model, &mut Data, usize, super::enums::SensorStage) + Send + Sync>;

/// User actuator dynamics callback: called for `ActuatorDynamics::User`.
///
/// Arguments: `(model, data, actuator_id)`.
/// Returns `act_dot` (activation derivative).
pub type CbActDyn = Callback<dyn Fn(&Model, &Data, usize) -> f64 + Send + Sync>;

/// User actuator gain callback: called for `GainType::User`.
///
/// Arguments: `(model, data, actuator_id)`.
/// Returns the gain value.
pub type CbActGain = Callback<dyn Fn(&Model, &Data, usize) -> f64 + Send + Sync>;

/// User actuator bias callback: called for `BiasType::User`.
///
/// Arguments: `(model, data, actuator_id)`.
/// Returns the bias value.
pub type CbActBias = Callback<dyn Fn(&Model, &Data, usize) -> f64 + Send + Sync>;
