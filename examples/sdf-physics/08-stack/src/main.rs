//! SDF Physics 08 — Stack
//!
//! Three cubes stacked on a ground plane. Tests multi-body simultaneous
//! contact stability before adding joints. If stacking is fragile, joints
//! will only make it worse.
//!
//! Isolates: simultaneous multi-body contact. Steps 07 tested two bodies;
//! this tests three with contacts at two interfaces plus ground.
//!
//! Pass criteria:
//! - All three cubes settle into a stable stack within 3 seconds
//! - No body passes through another
//! - Stack remains stable for at least 5 more seconds after settling
//! - No jitter, drift, or slow collapse
//!
//! New concept: multi-body stacking stability
//! Depends on: 07-pair
//!
//! Run with: `cargo run -p example-sdf-08-stack --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
