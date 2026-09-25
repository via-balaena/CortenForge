//! The experiment's fixtures: the geometry, loading and readouts of plan
//! §15's tube on a mandrel, and the oracle's golden values.
//!
//! A public module rather than a `test-fixtures` feature (plan §16f): code
//! behind a feature is neither coverage-measured nor doc-checked, and these
//! fixtures define what the kill criteria measure. `sim-gpu`'s conformance
//! tests and the benchmarks use them too.

pub mod golden;
pub mod tube;
