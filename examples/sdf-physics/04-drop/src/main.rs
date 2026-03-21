//! SDF Physics 04 — Drop
//!
//! A cf-design Solid dropped from 50mm onto the ground plane. It should fall,
//! impact, bounce (if restitution > 0), and settle.
//!
//! Isolates: dynamic contact response. Does impact at speed work? Is bounce
//! height physically reasonable? Any tunneling at impact velocity?
//!
//! Pass criteria:
//! - Body falls and hits the ground (doesn't pass through)
//! - Bounce height is reasonable (not higher than drop height)
//! - Body eventually settles to rest
//! - No explosion or instability at impact
//!
//! New concept: dynamic contact + restitution
//! Depends on: 03-rest (`sdf_plane_contact` works at rest)
//!
//! Run with: `cargo run -p example-sdf-04-drop --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
