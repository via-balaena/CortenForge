//! SDF Physics 09 — Hinge Stop (Parent-Child Collision)
//!
//! A hinged arm swings and hits the parent body's SDF geometry. This requires
//! `DISABLE_FILTERPARENT` so that parent-child geom pairs can collide.
//! Uses a flat stop surface on the base — single-contact `sdf_sdf_contact`
//! may suffice here since the geometry is convex at the contact region.
//!
//! Isolates: parent-child SDF-SDF collision. Everything before this only
//! collided non-parent-child pairs. This is the critical test for geometry-
//! as-constraint in articulated mechanisms.
//!
//! Pass criteria:
//! - Arm swings and hits the base body's geometry
//! - SDF collision stops the arm (geometry is the constraint, not joint limits)
//! - No vibration or tunneling at the contact
//! - Arm settles against the base
//!
//! New concept: parent-child SDF-SDF contact (`DISABLE_FILTERPARENT`)
//! Depends on: 08-hinge-wall (articulated contact works with external bodies)
//!
//! Run with: `cargo run -p example-sdf-09-hinge-stop --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
