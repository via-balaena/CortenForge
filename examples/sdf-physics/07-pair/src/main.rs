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
//! Blocked: surface-tracing algorithm produces equatorial contacts (horizontal
//! normals) instead of polar contacts (vertical normals) at 1 mm grid resolution.
//! See `sim/docs/SDF_SDF_CONTACT_SPEC.md` for details.
//!
//! Run with: `cargo run -p example-sdf-07-pair --release`

fn main() {
    eprintln!("BLOCKED: SDF-SDF contact at 1mm resolution produces horizontal normals");
    eprintln!("The surface-tracing algorithm finds contacts around the equatorial ring");
    eprintln!("of the overlap region, not at the vertical stacking point.");
    eprintln!("See sim/docs/SDF_SDF_CONTACT_SPEC.md");
}
