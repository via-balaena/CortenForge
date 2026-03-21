//! SDF Physics 04 — Rest
//!
//! A cf-design Solid placed just above a ground plane. It should settle onto
//! the ground and come to rest.
//!
//! Isolates: does `sdf_plane_contact` work? Does the contact normal point the
//! right way? Does the body reach equilibrium?
//!
//! Pass criteria:
//! - Body settles onto ground without passing through
//! - Velocity approaches zero within a few seconds
//! - No jitter or vibration at rest
//! - No visible penetration into the ground plane
//!
//! New concept: `sdf_plane_contact`
//! Depends on: 03-freefall
//!
//! Run with: `cargo run -p example-sdf-04-rest --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
