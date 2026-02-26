//! Warning tracking system for simulation diagnostics.
//!
//! Matches MuJoCo's `mjtWarning` enum and per-warning statistics.
//! Warnings are accumulated in `Data.warnings` and can be queried
//! after each step.

use super::data::Data;

/// Warning types (matches MuJoCo's `mjtWarning` enum).
/// `repr(u8)` for compact storage; cast to `usize` for array indexing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Warning {
    /// Bad inertia (non-positive definite).
    Inertia = 0,
    /// Contact buffer full.
    ContactFull = 1,
    /// Constraint buffer full.
    ConstraintFull = 2,
    /// Visual geom buffer full.
    VgeomFull = 3,
    /// Bad qpos (NaN/Inf/diverged).
    BadQpos = 4,
    /// Bad qvel (NaN/Inf/diverged).
    BadQvel = 5,
    /// Bad qacc (NaN/Inf/diverged).
    BadQacc = 6,
    /// Bad ctrl (NaN/Inf/diverged).
    BadCtrl = 7,
}

/// Number of warning types.
pub const NUM_WARNINGS: usize = 8;

/// Per-warning statistics.
#[derive(Debug, Clone, Copy, Default)]
pub struct WarningStat {
    /// Index that triggered the warning (e.g., DOF index for BadQpos).
    pub last_info: i32,
    /// Cumulative count since last reset.
    pub count: i32,
}

/// Format a warning message for display.
fn warning_text(warning: Warning, info: i32) -> String {
    match warning {
        Warning::Inertia => format!("Warning: bad inertia at body {info}."),
        Warning::ContactFull => format!("Warning: contact buffer full (ncon={info})."),
        Warning::ConstraintFull => format!("Warning: constraint buffer full (nefc={info})."),
        Warning::VgeomFull => format!("Warning: visual geom buffer full ({info})."),
        Warning::BadQpos => format!("Warning: bad qpos at index {info}."),
        Warning::BadQvel => format!("Warning: bad qvel at index {info}."),
        Warning::BadQacc => format!("Warning: bad qacc at index {info}."),
        Warning::BadCtrl => format!("Warning: bad ctrl at index {info}."),
    }
}

/// Record a warning and log (on first occurrence only).
/// Matches MuJoCo's `mj_warning()`.
pub fn mj_warning(data: &mut Data, warning: Warning, info: i32) {
    let w = &mut data.warnings[warning as usize];
    if w.count == 0 {
        tracing::warn!("{} Time = {:.4}.", warning_text(warning, info), data.time);
    }
    w.last_info = info;
    w.count += 1;
}
