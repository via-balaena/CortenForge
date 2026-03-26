//! SDF Physics 13 — Hinge Stop (Parent-Child Collision)
//!
//! A hinged arm swings and hits the parent body's SDF geometry. This
//! requires `DISABLE_FILTERPARENT` so that parent-child geom pairs can
//! collide. Uses a flat stop surface — single-contact `sdf_sdf_contact`
//! should suffice for convex geometry at the contact region.
//!
//! Isolates: parent-child SDF-SDF collision. The only new variable is
//! `DISABLE_FILTERPARENT`. Geometry at the contact is convex (flat).
//!
//! Pass criteria:
//! - Arm swings and hits the base body's geometry
//! - SDF collision stops the arm (geometry is the constraint, not joint limits)
//! - No vibration or tunneling at the contact
//! - Arm settles against the base
//!
//! New concept: parent-child SDF-SDF contact (`DISABLE_FILTERPARENT`)
//! Depends on: 10-hinge-wall
//!
//! Run with: `cargo run -p example-sdf-cpu-13-hinge-stop --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
