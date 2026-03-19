//! Tendon-Driven Finger — Mechanism Assembly + Export
//!
//! Build a 3-phalanx finger using cf-design's template system, route a flexor
//! tendon through it, attach a motor actuator, and export to both STL (for 3D
//! printing) and MJCF (for physics simulation).
//!
//! This demonstrates the "design IS the simulation model" philosophy:
//! tendon channels are automatically subtracted from the geometry at build time.
//!
//! Run with: `cargo run -p example-tendon-finger`

use cf_design::{
    ActuatorDef, ActuatorKind, JointDef, JointKind, Material, Mechanism, TendonDef, TendonWaypoint,
    templates,
};
use nalgebra::{Point3, Vector3};

fn main() -> anyhow::Result<()> {
    println!("=== CortenForge: Tendon-Driven Finger ===\n");

    // ── Material ────────────────────────────────────────────────────────
    let pla = Material::new("PLA", 1250.0)
        .with_youngs_modulus(3.5e9)
        .with_color([0.9, 0.85, 0.7, 1.0]);

    // ── Finger from template ────────────────────────────────────────────
    // Creates 3 phalanges with flex zones at the knuckles.
    // Returns (parts, joints) — joints have spring-damper properties derived
    // from the material's Young's modulus.
    let (finger_parts, finger_joints) = templates::finger("finger", 30.0, 3.5, 3, pla.clone());

    println!("Finger template:");
    println!("  {} phalanges", finger_parts.len());
    println!("  {} flex joints", finger_joints.len());
    for j in &finger_joints {
        println!(
            "    {} ({} -> {}), stiffness={:.0}",
            j.name(),
            j.parent(),
            j.child(),
            j.stiffness().unwrap_or(0.0),
        );
    }

    // ── Palm (mount point) ──────────────────────────────────────────────
    let palm = cf_design::Part::new(
        "palm",
        cf_design::Solid::cuboid(Vector3::new(8.0, 12.0, 4.0)).round(1.0),
        pla,
    );

    // ── Assemble mechanism ──────────────────────────────────────────────
    let mut builder = Mechanism::builder("tendon_finger").part(palm);

    for part in finger_parts {
        builder = builder.part(part);
    }

    // Connect palm to proximal phalanx
    builder = builder.joint(
        JointDef::new(
            "knuckle",
            "palm",
            "finger_0",
            JointKind::Revolute,
            Point3::new(0.0, 12.0, 0.0),
            Vector3::x(),
        )
        .with_range(-0.1, 1.8),
    );

    // Add inter-phalanx joints from the template
    for j in finger_joints {
        builder = builder.joint(j);
    }

    // ── Tendon routing ──────────────────────────────────────────────────
    // Route a flexor tendon from the palm base, through the palm, and into
    // the finger. Channel radius > 0 means the builder will automatically
    // subtract a cylindrical channel from each part the tendon passes through.
    builder = builder.tendon(
        TendonDef::new(
            "flexor",
            vec![
                TendonWaypoint::new("palm", Point3::new(0.0, -8.0, 0.0)),
                TendonWaypoint::new("palm", Point3::new(0.0, 10.0, 0.0)),
                TendonWaypoint::new("finger_0", Point3::new(0.0, 0.0, 2.0)),
                TendonWaypoint::new("finger_1", Point3::new(0.0, 0.0, 2.0)),
                TendonWaypoint::new("finger_2", Point3::new(0.0, 0.0, 1.0)),
            ],
            1.0, // channel radius (mm)
        )
        .with_stiffness(200.0)
        .with_damping(10.0),
    );

    // ── Actuator ────────────────────────────────────────────────────────
    builder = builder.actuator(
        ActuatorDef::new("motor", "flexor", ActuatorKind::Motor, (-50.0, 50.0))
            .with_ctrl_range(-1.0, 1.0),
    );

    // ── Build (validates + subtracts tendon channels) ────────────────────
    let mechanism = builder.build();
    println!("\nMechanism '{}' built successfully.", mechanism.name());
    println!("  {} parts", mechanism.parts().len());
    println!("  {} joints", mechanism.joints().len());
    println!("  {} tendons", mechanism.tendons().len());
    println!("  {} actuators", mechanism.actuators().len());

    // ── Export MJCF (for simulation) ────────────────────────────────────
    let mjcf = mechanism.to_mjcf(1.0);
    std::fs::write("tendon_finger.mjcf", &mjcf)?;
    println!("\n  tendon_finger.mjcf — {} bytes", mjcf.len());

    // ── Export STL kit (for 3D printing) ────────────────────────────────
    let stl_kit = mechanism.to_stl_kit(0.5);
    for (name, mesh) in &stl_kit {
        let path = format!("{name}.stl");
        mesh_io::save_stl(mesh, &path, true)?;
        println!(
            "  {name}.stl — {} vertices, {} faces",
            mesh.vertices.len(),
            mesh.faces.len()
        );
    }

    println!("\nFinger exported for both simulation and printing.");
    Ok(())
}
