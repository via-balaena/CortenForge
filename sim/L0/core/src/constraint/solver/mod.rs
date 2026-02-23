//! Constraint solver dispatch and shared infrastructure.
//!
//! Routes to the configured solver algorithm (PGS, CG, Newton) and provides
//! shared post-solve operations (Delassus regularization, qfrc recovery,
//! friction pyramid decoding). Corresponds to MuJoCo's `engine_solver.c`.
