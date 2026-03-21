//! SDF Physics 12 — Damped Hinge (Parameter Sensitivity)
//!
//! Same geometry as step 11, run with three different damping/contact
//! parameter sets. Proves we understand the tuning knobs before attempting
//! the socket.
//!
//! Three configurations:
//! 1. High damping — arm reaches stop slowly, no bounce
//! 2. Low damping, stiff contact — arm swings freely, bounces crisply
//! 3. Medium damping — arm settles in 2-3 oscillations
//!
//! Isolates: damping and contact parameter (`solref`, `solimp`) sensitivity.
//! Steps 09-11 used defaults. This verifies that changing them has the
//! expected effect.
//!
//! Pass criteria:
//! - All three parameter sets produce stable simulation
//! - Settling time ordering: high < medium < low damping
//! - No parameter set causes explosion or tunneling
//!
//! New concept: damping + contact parameter tuning
//! Depends on: 11-hinge-stop
//!
//! Run with: `cargo run -p example-sdf-12-damped-hinge --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
