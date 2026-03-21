//! SDF Physics 03 — Freefall
//!
//! A cf-design Solid converted to a physics body via `to_model()`. No ground
//! plane — the body free-falls under gravity.
//!
//! Isolates: does `to_model()` produce a body with correct mass/inertia?
//! Does gravity work at mm scale?
//!
//! Pass criteria:
//! - Body has non-zero mass and reasonable inertia
//! - Position follows analytical freefall: `z(t) = z0 - 0.5*g*t^2`
//! - No unexpected rotation (symmetric body, no torque)
//!
//! New concept: `to_model()` pipeline + gravity at mm scale
//! Depends on: 01-sdf-grid, 02-thin-grid
//!
//! Run with: `cargo run -p example-sdf-03-freefall --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
