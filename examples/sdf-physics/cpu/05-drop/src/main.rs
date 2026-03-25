//! SDF Physics 05 — Drop
//!
//! A cf-design Solid dropped from 50 mm onto the ground plane. It should
//! fall, impact, bounce, and settle.
//!
//! Isolates: dynamic contact response at impact velocity. Is bounce height
//! physically reasonable? Any tunneling?
//!
//! Pass criteria:
//! - Body falls and hits the ground (doesn't pass through)
//! - Bounce height < drop height (energy dissipation)
//! - Body eventually settles to rest
//! - No explosion or instability at impact
//!
//! New concept: dynamic contact + restitution
//! Depends on: 04-rest
//!
//! Run with: `cargo run -p example-sdf-cpu-05-drop --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
