//! Obstacle-Avoiding Robot — Phase 3 Product
//!
//! Wheeled robot with range sensor array, occupancy grid mapping,
//! and real-time A* path planning using CortenForge crates.

#[allow(clippy::unnecessary_wraps)]
fn main() -> anyhow::Result<()> {
    println!("=== CortenForge Example: Obstacle-Avoiding Robot ===");
    println!();
    println!("Phase 3: Perception + Control Loop");
    println!("Status: Not yet implemented");
    println!();
    println!("Pipeline:");
    println!("  1. Sense  — Read range sensors (ultrasonic/IR array)");
    println!("  2. Map    — Update local occupancy grid");
    println!("  3. Plan   — A* path to goal through free space");
    println!("  4. Act    — Differential-drive motor commands");
    println!("  5. Replan — Re-path on new obstacle detection");
    println!();
    println!("Modes:");
    println!("  --sim      Simulated range sensors via sim-sensor (default)");
    println!("  --hardware Real ultrasonic/IR sensors");

    Ok(())
}
