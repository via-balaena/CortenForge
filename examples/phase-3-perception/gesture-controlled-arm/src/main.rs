//! Gesture-Controlled Arm — Phase 3 Product
//!
//! IMU-instrumented glove teleoperating a 3-DOF robot arm using
//! CortenForge sensor-fusion and trajectory planning crates.

#[allow(clippy::unnecessary_wraps)]
fn main() -> anyhow::Result<()> {
    println!("=== CortenForge Example: Gesture-Controlled Arm ===");
    println!();
    println!("Phase 3: Perception + Control Loop");
    println!("Status: Not yet implemented");
    println!();
    println!("Pipeline:");
    println!("  1. Sense — Read glove IMU (accel + gyro)");
    println!("  2. Fuse  — Orientation estimation -> wrist pose");
    println!("  3. Map   — Wrist orientation -> target joint angles");
    println!("  4. Plan  — Collision-free joint trajectory");
    println!("  5. Act   — Command servos (shoulder, elbow, wrist)");
    println!();
    println!("Modes:");
    println!("  --sim      Simulated glove IMU via sim-sensor (default)");
    println!("  --hardware Real IMU glove + servos");

    Ok(())
}
