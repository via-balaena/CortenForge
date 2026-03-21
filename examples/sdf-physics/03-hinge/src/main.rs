//! SDF Physics 03 — Hinge
//!
//! Two parts connected by a revolute joint. The arm swings under gravity
//! and its SDF collides with the base's SDF (parent-child collision).
//! Proves: `DISABLE_FILTERPARENT` + multi-contact `sdf_sdf_contact` works.
//!
//! Pass criteria:
//! - Arm swings freely until it hits the base geometry
//! - SDF collision stops the arm (geometry is the constraint, not joint limits)
//! - No vibration or tunneling at the contact
//!
//! Depends on: 02-pair working.
//! Blocked by: `sdf_sdf_contact` returns only 1 contact (needs multi-contact).
//!
//! Run with: `cargo run -p example-sdf-03-hinge --release`

fn main() {
    eprintln!("BLOCKED: needs multi-contact `sdf_sdf_contact` — see examples/EXAMPLES.md");
}
