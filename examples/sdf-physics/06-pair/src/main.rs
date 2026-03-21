//! SDF Physics 06 — Pair
//!
//! Two cf-design Solids. One sits on the ground, the other drops onto it.
//! First test of SDF-vs-SDF collision (non-parent-child, independent bodies).
//!
//! Isolates: `sdf_sdf_contact`. Everything before this only tested SDF-vs-plane.
//! Now two discretized distance fields must find their overlap and generate
//! contact forces.
//!
//! Pass criteria:
//! - Upper body falls and lands on the lower body
//! - No tunneling — they don't pass through each other
//! - Contact forces are reasonable (no explosion)
//! - Both eventually settle to rest (stacked)
//!
//! New concept: `sdf_sdf_contact`
//! Depends on: 05-slide (SDF-plane contact is solid in all directions)
//!
//! Run with: `cargo run -p example-sdf-06-pair --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
