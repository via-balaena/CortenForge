//! SDF Physics 07 — Pair
//!
//! Two cf-design Solids. One sits on the ground, the other drops onto it.
//! First test of SDF-vs-SDF collision (non-parent-child, independent bodies).
//!
//! Isolates: `sdf_sdf_contact`. Everything before this only tested SDF-vs-plane.
//!
//! Pass criteria:
//! - Upper body falls and lands on the lower body
//! - No tunneling — they don't pass through each other
//! - Contact forces are reasonable (no explosion)
//! - Both eventually settle to rest (stacked)
//!
//! New concept: `sdf_sdf_contact`
//! Depends on: 06-slide
//!
//! Blocked: SDF-SDF contact algorithm produces correct contacts (normals,
//! penetration) but the constraint solver doesn't maintain separation across
//! timesteps. See `sim/docs/SDF_SDF_CONTACT_SPEC.md` for investigation notes.
//!
//! Run with: `cargo run -p example-sdf-07-pair --release`

fn main() {
    eprintln!("BLOCKED: SDF-SDF stacking requires constraint solver investigation");
    eprintln!("Contact algorithm is correct (stress tests pass), but the solver");
    eprintln!("doesn't maintain separation across multiple timesteps.");
    eprintln!("See sim/docs/SDF_SDF_CONTACT_SPEC.md");
}
