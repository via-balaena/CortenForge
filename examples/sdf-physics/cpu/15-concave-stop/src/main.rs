//! SDF Physics 15 — Concave Stop (Half-Socket)
//!
//! A concave pocket on the parent body. The arm tip enters the curved pocket.
//! This is a **designed failure point**: if single-contact `sdf_sdf_contact`
//! can't handle concavity, the arm will escape laterally. This tells us
//! exactly when to invest in multi-contact — before step 14, not during it.
//!
//! Isolates: concave parent geometry constraining the child. Step 11 used
//! a flat (convex) stop. This uses a curved half-socket.
//!
//! Pass criteria:
//! - Arm swings and enters the concave pocket
//! - Contact keeps the arm tip within the pocket
//! - If single-contact fails: arm escapes laterally (clear, diagnosable)
//! - If single-contact works: arm settles stably in the pocket
//!
//! New concept: concave parent-child SDF contact
//! Depends on: 12-damped-hinge
//!
//! Run with: `cargo run -p example-sdf-cpu-15-concave-stop --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
