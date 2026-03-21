//! SDF Physics 01 — SDF Grid
//!
//! Pure cf-design, no simulation. Creates a Solid, builds an `SdfGrid` from it,
//! and visualizes the resulting mesh. Prints grid dimensions, cell count, and
//! min/max distance values to verify the SDF field is sane.
//!
//! This is the very first step: does `Solid → SdfGrid` even produce valid data?
//!
//! Pass criteria:
//! - `SdfGrid` is constructed without panic
//! - Grid dimensions and cell count are physically reasonable
//! - Min distance is negative (interior exists)
//! - Max distance is positive (exterior exists)
//! - Visual mesh looks correct when orbiting
//!
//! New concept: Solid → `SdfGrid` construction
//!
//! Run with: `cargo run -p example-sdf-01-sdf-grid --release`

fn main() {
    eprintln!("TODO: not yet implemented — see examples/EXAMPLES.md for roadmap");
}
