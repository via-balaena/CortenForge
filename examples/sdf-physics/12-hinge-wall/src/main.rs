//! SDF Physics 12 — Hinge + Wall
//!
//! A hinged arm swings and hits a separate wall body. The wall is NOT the
//! parent — it's an independent SDF body. This tests articulated contact
//! with external geometry.
//!
//! Isolates: contact during articulated motion against external geometry.
//! The arm is on a joint but the collision is non-parent-child.
//!
//! Pass criteria:
//! - Arm swings under gravity and hits the wall
//! - Arm stops at the wall (doesn't pass through)
//! - No instability at the contact
//! - Arm may bounce/settle against the wall
//!
//! New concept: articulated contact with external geometry
//! Depends on: 09-hinge-free
//!
//! Run with: `cargo run -p example-sdf-12-hinge-wall --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
