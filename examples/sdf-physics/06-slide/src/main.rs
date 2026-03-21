//! SDF Physics 06 — Slide
//!
//! A cf-design Solid on the ground with an initial lateral velocity. It should
//! slide along the surface, decelerate due to friction, and come to rest.
//!
//! Isolates: tangential contact forces / friction. Everything before this only
//! tested normal (vertical) forces.
//!
//! Pass criteria:
//! - Body slides horizontally along the ground
//! - Deceleration is visible (friction is working)
//! - Body comes to rest (doesn't slide forever)
//! - No lift-off or wobble during slide
//!
//! New concept: friction / tangential contact forces
//! Depends on: 05-drop
//!
//! Run with: `cargo run -p example-sdf-06-slide --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
