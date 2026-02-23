//! Equality constraint Jacobian extraction.
//!
//! Computes Jacobian rows for connect, weld, joint, and distance equality
//! constraints. Each `extract_*_jacobian` function fills one or more rows
//! of the unified constraint arrays.
