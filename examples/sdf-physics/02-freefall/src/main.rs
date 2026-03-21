//! SDF Physics 02 — Freefall
//!
//! A cf-design Solid converted to a physics body via `to_model()`. No ground
//! plane — the body free-falls under gravity. Prints position each frame and
//! verifies z(t) matches the analytical solution.
//!
//! Uses a free joint to "world" so the body is unconstrained.
//!
//! Isolates: does `to_model()` produce a body with correct mass/inertia?
//! Does gravity work at mm scale?
//!
//! Pass criteria:
//! - Body appears with non-zero mass and reasonable inertia
//! - Position follows analytical freefall curve
//! - No unexpected rotation (symmetric body, no torque)
//!
//! New concept: `to_model()` pipeline + gravity at mm scale
//! Depends on: 01-sdf-grid (`SdfGrid` construction works)
//!
//! Run with: `cargo run -p example-sdf-02-freefall --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
