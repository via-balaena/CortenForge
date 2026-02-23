//! Constraint row assembly for all constraint types.
//!
//! Populates the unified `efc_*` arrays on [`Data`] with constraint Jacobians,
//! parameters, and metadata for equality, friction, limit, and contact
//! constraints. Corresponds to MuJoCo's `engine_core_constraint.c`.
