//! SDF Physics 07 — Hinge (Free Swing)
//!
//! Two SDF bodies connected by a revolute joint. The arm swings freely under
//! gravity. No collision between the arm and the base — this only tests that
//! joints work correctly with SDF bodies.
//!
//! Isolates: joint integration with SDF geom types. All previous steps used
//! free bodies. This is the first step with articulation.
//!
//! Pass criteria:
//! - Arm swings smoothly on the hinge
//! - No explosion or divergence
//! - Energy is approximately conserved (small damping OK)
//! - Joint axis is correct (arm swings in the expected plane)
//!
//! New concept: joints with SDF bodies
//! Depends on: 06-pair (SDF-SDF contact works for free bodies)
//!
//! Run with: `cargo run -p example-sdf-07-hinge-free --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
